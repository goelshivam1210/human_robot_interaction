#######################################################################
# Makefile for python test-suite
#######################################################################

ifeq (,$(PY3))
  PYBIN = python
else
  PYBIN = python3
endif

LANGUAGE     = python
PYTHON       = $(PYBIN)
PYCODESTYLE       = 
PYCODESTYLE_FLAGS = --ignore=E252,E30,E402,E501,E731,W291,W391

#*_runme.py for Python 2.x, *_runme3.py for Python 3.x
PY2SCRIPTSUFFIX = _runme.py
PY3SCRIPTSUFFIX = _runme3.py
PY2TO3       = 2to3 -x import

ifeq (,$(PY3))
  SCRIPTSUFFIX = $(PY2SCRIPTSUFFIX)
else
  SCRIPTSUFFIX = $(PY3SCRIPTSUFFIX)
endif

srcdir       = .
top_srcdir   = ../../..
top_builddir = ../../..

CPP_TEST_CASES += \
	argcargvtest \
	callback \
	complextest \
	director_stl \
	director_wstring \
	file_test \
	iadd \
	implicittest \
	inout \
	inplaceadd \
	input \
	li_cstring \
	li_cwstring \
	li_factory \
	li_implicit \
	li_std_containers_int \
	li_std_list \
	li_std_map_member \
	li_std_multimap \
	li_std_pair_extra \
	li_std_set \
	li_std_stream \
	li_std_string_extra \
	li_std_vectora \
	li_std_vector_extra \
	li_std_wstream \
	li_std_wstring \
	primitive_types \
	python_abstractbase \
	python_append \
	python_builtin \
	python_destructor_exception \
	python_director \
	python_docstring \
	python_extranative \
	python_moduleimport \
	python_overload_simple_cast \
	python_pickle \
	python_pybuffer \
	python_pythoncode \
	python_richcompare \
	python_strict_unicode \
	python_threads \
	simutry \
	std_containers \
	swigobject \
	template_matrix \

#	li_std_carray
#	director_profile

CPP11_TEST_CASES = \
	cpp11_hash_tables \
	cpp11_shared_ptr_const \
	cpp11_shared_ptr_nullptr_in_containers \
	cpp11_shared_ptr_overload \
	cpp11_shared_ptr_upcast \
	cpp11_std_unordered_map \
	cpp11_std_unordered_multimap \
	cpp11_std_unordered_multiset \
	cpp11_std_unordered_set \

C_TEST_CASES += \
	file_test \
	li_cstring \
	li_cwstring \
	python_nondynamic \
	python_varargs_typemap \

#
# This test only works with modern C compilers
#
#C_TEST_CASES += \
#	complextest

include $(srcdir)/../common.mk

# Overridden variables here
SCRIPTDIR    = .
LIBS         = -L.
VALGRIND_OPT += --suppressions=pythonswig.supp

# Custom tests - tests with additional commandline options
# none!

# Rules for the different types of tests
%.cpptest:
	+$(convert_testcase)
	$(setup)
	+$(swig_and_compile_cpp)
	$(check_pep8)
	$(run_testcase)

%.ctest:
	+$(convert_testcase)
	$(setup)
	+$(swig_and_compile_c)
	$(check_pep8)
	$(run_testcase)

%.multicpptest:
	+$(convert_testcase)
	$(setup)
	+$(swig_and_compile_multi_cpp)
	$(check_pep8_multi_cpp)
	$(run_testcase)



# Runs the testcase. A testcase is only run if
# a file is found which has _runme.py (or _runme3.py for Python 3) appended after the testcase name.

py_runme  = $(SCRIPTPREFIX)$*$(SCRIPTSUFFIX)
py2_runme = $(SCRIPTPREFIX)$*$(PY2SCRIPTSUFFIX)
py3_runme = $(SCRIPTPREFIX)$*$(PY3SCRIPTSUFFIX)

ifneq (,$(PYCODESTYLE))
check_pep8 = $(COMPILETOOL) $(PYCODESTYLE) $(PYCODESTYLE_FLAGS) $(SCRIPTPREFIX)$*.py

check_pep8_multi_cpp = \
	for f in `cat $(top_srcdir)/$(EXAMPLES)/$(TEST_SUITE)/$*.list` ; do \
	  $(COMPILETOOL) $(PYCODESTYLE) $(PYCODESTYLE_FLAGS) $$f.py; \
	done
endif

run_python = env LD_LIBRARY_PATH=.:$$LD_LIBRARY_PATH PYTHONPATH=.:$(srcdir):$$PYTHONPATH $(RUNTOOL) $(PYTHON) $(py_runme)

run_testcase = \
	if [ -f $(SCRIPTDIR)/$(py_runme) ]; then \
	  $(run_python);\
	fi

# Grab runme file ready for running: copied for out of source tree builds, and/or run 2to3
# Note terminal (double colon) rules creating runme files to fix possible infinite recursion,
# see https://github.com/swig/swig/pull/688
ifeq ($(SCRIPTDIR),$(srcdir))
  # in source tree build
  ifeq (,$(PY3))
    convert_testcase =
  else
    convert_testcase = \
	if [ -f $(srcdir)/$(py2_runme) ]; then \
	  $(MAKE) $(SCRIPTDIR)/$(py_runme); \
	fi

# For converting python 2 tests into Python 3 tests
$(SCRIPTDIR)/$(SCRIPTPREFIX)%$(SCRIPTSUFFIX):: $(srcdir)/$(SCRIPTPREFIX)%$(PY2SCRIPTSUFFIX)
	cp $< $@
	$(PY2TO3) -w $@ >/dev/null 2>&1

  endif
else
  # out of source tree build
  ifeq (,$(PY3))
    convert_testcase = \
	if [ -f $(srcdir)/$(py2_runme) ]; then \
	  $(MAKE) $(SCRIPTDIR)/$(py_runme); \
	fi

$(SCRIPTDIR)/$(SCRIPTPREFIX)%$(SCRIPTSUFFIX):: $(srcdir)/$(SCRIPTPREFIX)%$(PY2SCRIPTSUFFIX)
	cp $< $@

  else
    convert_testcase = \
	if [ -f $(srcdir)/$(py2_runme) ]; then \
	  $(MAKE) $(SCRIPTDIR)/$(py_runme); \
	elif [ -f $(srcdir)/$(py3_runme) ]; then \
	  $(MAKE) $(SCRIPTDIR)/$(py3_runme); \
	fi

# For when there is a _runme3.py instead of a _runme.py, ie a Python 3 only run test
$(SCRIPTDIR)/$(SCRIPTPREFIX)%$(SCRIPTSUFFIX):: $(srcdir)/$(SCRIPTPREFIX)%$(PY3SCRIPTSUFFIX)
	cp $< $@

# For converting python 2 tests into Python 3 tests
$(SCRIPTDIR)/$(SCRIPTPREFIX)%$(SCRIPTSUFFIX):: $(srcdir)/$(SCRIPTPREFIX)%$(PY2SCRIPTSUFFIX)
	cp $< $@
	$(PY2TO3) -w $@ >/dev/null 2>&1

  endif

endif

# Clean: remove the generated .py file
# We only remove the _runme3.py if it is generated by 2to3 from a _runme.py.
%.clean:
	@rm -f $*.py
	@if test -f $(srcdir)/$(py2_runme); then rm -f $(SCRIPTDIR)/$(py3_runme) $(SCRIPTDIR)/$(py3_runme).bak; fi
	@if test "x$(SCRIPTDIR)" != "x$(srcdir)"; then rm -f $(SCRIPTDIR)/$(py_runme); fi

clean:
	$(MAKE) -f $(top_builddir)/$(EXAMPLES)/Makefile SRCDIR='$(SRCDIR)' python_clean
	rm -f hugemod.h hugemod_a.i hugemod_b.i hugemod_a.py hugemod_b.py hugemod_runme.py
	rm -f clientdata_prop_a.py clientdata_prop_b.py import_stl_a.py import_stl_b.py
	rm -f imports_a.py imports_b.py mod_a.py mod_b.py multi_import_a.py
	rm -f multi_import_b.py packageoption_a.py packageoption_b.py packageoption_c.py

hugemod_runme = hugemod$(SCRIPTPREFIX)

hugemod:
	perl hugemod.pl $(hugemod_runme)
	$(MAKE) hugemod_a.cpptest
	$(MAKE) hugemod_b.cpptest
	sh -c "time $(PYTHON) $(hugemod_runme)"
	sh -c "time $(PYTHON) $(hugemod_runme)"
