import warnings

import numpy as np

from dynfil.commands import main, filter, compare, evaluate  # NOQA

warnings.simplefilter("ignore", category=np.RankWarning)

if __name__ == '__main__':
    main.main(obj={})
