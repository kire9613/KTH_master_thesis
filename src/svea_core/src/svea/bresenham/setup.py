import distutils.core
import Cython.Build
distutils.core.setup(
    ext_modules = Cython.Build.cythonize("bresenham.pyx",
                                         build_dir="build"
                                         ))
