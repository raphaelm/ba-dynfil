=======================================================
BA-Thesis: Dynamic Filter for Walking Motion Correction
=======================================================

:Author:
:Address:
:Contact:
:organization: Interdisciplinary Center for Scientific Computing, Heidelberg University


Install
=======

The code is based on the fast and efficient computation of multi-body dynamics.
For this purpose, we use RBDL - an efficient rigid-body dynamics library using
recursive algorithms.
The library can be installed from::

  https://bitbucket.org/rbdl/rbdl

The detailed documentation can be found here::

  http://rbdl.bitbucket.org/


Usage
=====

Pattern Generator
-----------------

This repository comes with a very simple pattern generator that can be invoked with

	python patterngenerator.py

It will show two plots and output a trajectory to out/pg_data.txt

Main Script
-----------

The main script supports a number of commandline options and subcommands.
It can be invoked in the following way:

	python main.py [toplevel options ...] subcommand [command options ...]

Currently, the following values can be set as top-level options:

   --model TEXT          Model file
   --trajectory TEXT     Trajectory file  [required]
   --csv-delim TEXT      CSV delimiter of trajectory file
   --out-dir TEXT        Output directory
   --show                Open plot windows
   -w / --show-warnings  Show warnings
   --help                Show this message and exit.

There are three subcommands at this point in time. The most important one is
``filter``, which performs the full set of actions required to evaluate the
results of the dynamic filter, that means it:

* Calculates the inverse kinematics on the pattern generator data and calculates
  the ZMP trajectory from that using inverse dynamics

* Applies the dynamic filter

* Calculates a ZMP trajectory again

* Creates MeshUp animation files

* Creates a lot of plots.

It again takes a number of options, as a numbre of interpolation and filter
methods are currently implemented.


    --filter-method [newton|leastsquares|steepestdescent|pc]
                                    Filter method
    --ik-method [numerical|analytical]
                                    IK method
    --iterations INTEGER RANGE      Number of filter iterations
    --interpolate [none|poly|savgol]
                                    Apply interpolation

The other subcommands, ``compare_ik`` and ``compare_interpolation`` are for 
evaluation purposes of parts of the codebase. You can get more information
on their options using e.g.

    python main.py --trajectory out/pc_data.txt compare_interpolation --help


Examples
--------

An example that uses the data from the internal pattern generator and then applies
one iteration of the preview-control-based filter would be:

    python main.py --trajectory out/pg_data.txt --show filter --iterations 1 --filter-method pc --interpolate savgol

Or as a different example, with five Newton-Raphson iterations:

    python main.py --trajectory out/pg_data.txt --show filter --iterations 5 --filter-method newton --interpolate savgol
