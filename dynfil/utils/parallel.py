import copy
import multiprocessing

import importlib
import numpy as np

from ..bodies import BodyTrajectory


def _worker(module, name, **kwargs):
    mod = importlib.import_module(module)
    func = getattr(mod, name)
    if not hasattr(func, '_orig'):
        raise TypeError('This is not a parallelized function.')
    res = func._orig(**kwargs)
    print(res)
    return res


def parallelize(timed_args):
    """
    Usage:
    @parallel.parallelize(timed_args=('chest', 'lsole', 'rsole', 'zmp_ref', 'times'))
    """
    processcount = multiprocessing.cpu_count()

    def wrapper(func):
        def substitute(*args, **kwargs):
            if args:
                raise TypeError('Please only use keyword arguments to call this method.')

            # Split chunks
            chunks = {}
            all_len = None
            for argname in timed_args:
                if argname in kwargs:
                    val = kwargs[argname]
                    if all_len is None:
                        all_len = len(val)
                    elif all_len != len(val):
                        raise TypeError(
                            'Argument {} is not of length {} but of length {}. '
                            'All timed arguments need to be of the same length.'.format(
                                argname, all_len, len(val)
                            )
                        )

                    if isinstance(val, np.ndarray):
                        chunks[argname] = np.array_split(val, processcount)
                    elif isinstance(val, BodyTrajectory):
                        pos_split = np.array_split(val.traj_pos, processcount)
                        ort_split = np.array_split(val.traj_ort, processcount)
                        chunks[argname] = []
                        for i in range(processcount):
                            splitval = val.copy()
                            splitval.traj_pos = pos_split[i]
                            splitval.traj_ort = ort_split[i]
                            chunks[argname].append(splitval)
                    else:
                        raise TypeError('Unsupported argument type for {}: {}'.format(argname, type(val)))

            pool = multiprocessing.Pool(processcount)
            results = []
            for i in range(processcount):
                _kwargs = copy.copy(kwargs)
                for chunk, values in chunks.items():
                    if chunk in _kwargs:
                        _kwargs[chunk] = values[i]

                results.append(pool.apply_async(_worker, args=(func.__module__, func.__name__), kwds=_kwargs))

            try:
                pool.close()
                pool.join()
                results = [r.get(timeout=1) for r in results]
            except KeyboardInterrupt:
                pool.terminate()
                raise KeyboardInterrupt()

            if isinstance(results[0], BodyTrajectory):
                res = results[0]
                res.traj_pos = np.concatenate([r.traj_pos for r in results])
                res.traj_ort = np.concatenate([r.traj_ort for r in results])
                return res
            elif isinstance(results[0], np.ndarray):
                return np.concatenate(results)
            else:
                raise TypeError('Unsupported return type: {}'.format(type(results[0])))

        substitute._orig = func
        return substitute
    return wrapper
