import warnings

import numpy as np

from dynfil.commands import main, filter, compare  # NOQA

warnings.simplefilter("ignore", category=np.RankWarning)

if __name__ == '__main__':
    main.main(obj={})
