#!/bin/bash
set -ev
rm __init__.py
rm __init__.pyc
python2 setup.py build_ext --inplace
touch __init__.py
