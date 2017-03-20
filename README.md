# Dynamic Filter for Walking Motion Correction

This repository contains all code used to produce the results of my Bachelor's thesis.

The code works, but is **not actively maintained**.
Thanks to the friendly folks at [ORB](https://orb.uni-hd.de) for helping me make this happen.

## Installation

* Install Python 2.7, virtualenv and pip

* Create and activate a virtual environment and install Python-level dependencies with
  ``pip install -r requirements.txt``

* Install custom RBDL version with the modified IK and improved Python wrapper by K. Stein,
  M. Kudruss, P. Manns and myself from the dev branch available here:
  https://bitbucket.org/mkudruss/rbdl/

  If you are on Arch Linux, install the ``lua51`` package and run cmake with the following options:

      cmake .. -DRBDL_BUILD_ADDON_LUAMODEL=ON -DRBDL_BUILD_ADDON_URDFREADER=ON \
               -DRBDL_BUILD_PYTHON_WRAPPER=ON -DLUA_INCLUDE_DIR=/usr/include/lua5.1 \

  Make sure to execute cmake and make within your Python virtualenv, then the python
  versions will be automatically correct.

* Set your PYTHONPATH such that it includes the build/python/ directory from RBDL, e.g.
  ``export PYTHONPATH=../rbdl/build/python:$PYTHONPATH``

## Usage

The repository contains two executable Python scripts, ``patterngenerator.py`` and
``main.py``. Both take a similar set of options.

Currently, the following models are available in this repository:

* ``heicub`` -- A model of the HeiCub robot available at ORB

* ``simple`` -- A model of a very simple walking robot

* ``simplelx2`` -- Same as ``simple``, but with 2x heavier legs

* ``simplelx5`` -- Same as ``simple``, but with 5x heavier legs

* ``simplelx10`` -- Same as ``simple``, but with 10x heavier legs

### Pattern Generator

This repository comes with a very simple pattern generator that can be invoked with

	python patterngenerator.py [options ...]

It has the following options:

    --model [simple|heicub|simplelx5|simplelx10|simplelx2]
                                    Model
    --out-dir TEXT                  Output directory
    --help                          Show this message and exit.

It will output plots and a trajectory file to ``out/pg_data.txt`` by default.

### Main Script

The main script supports a number of commandline options and subcommands.
It can be invoked in the following way:

	python main.py [toplevel options ...] subcommand [command options ...]

Currently, the following values can be set as top-level options:

    --model [simple|heicub|simplelx5|simplelx10|simplelx2]
                                    Model
    --trajectory TEXT               Trajectory file  [required]
    --csv-delim TEXT                CSV delimiter of trajectory file
    --out-dir TEXT                  Output directory
    --show                          Open plot windows
    -w / --show-warnings            Show warnings
    --help                          Show this message and exit.

There are multiple subcommands. The most important one is ``filter``, which performs
the full set of actions required to evaluate the results of the dynamic filter, that
means it:

* Calculates the inverse kinematics on the pattern generator data and calculates
  the ZMP trajectory from that using inverse dynamics

* Applies the dynamic filter

* Calculates a ZMP trajectory again

* Creates MeshUp animation files

* Creates a lot of plots.

It again takes a number of options, as a number of interpolation and filter
methods are currently implemented.

    --filter-method [steepestdescent|gaussnewton|newton|pc]
                                    Filter method
    --ik-method [numerical|analytical]
                                    IK method
    --iterations INTEGER RANGE      Number of filter iterations
    --interpolate [none|savgol]     Apply interpolation to filter result
    --help                          Show this message and exit.

The other subcommands, ``compare_ik`` and ``compare_interpolation`` are for 
evaluation purposes of parts of the codebase.  ``evaluate`` creates plots for
a huge number of filter configurations at once, ``evaluate_speed`` evaluates
the performance of the algorithms, ``evaluate_zmp_accuracy`` compares two
different ZMP computation algorithms and ``plot_error`` creates a plot of the
raw result without any filters applied.

You can get more information on their options using e.g.

    python main.py --trajectory out/pc_data.txt compare_interpolation --help

### Makefile

The Makefile included runs a number of batch tasks that together generate (nearly)
all plots used in my thesis -- and a lot more.
