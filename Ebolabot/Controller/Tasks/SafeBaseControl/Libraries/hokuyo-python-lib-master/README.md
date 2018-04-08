hokuyo-python-lib
=================

[![Codacy Badge](https://api.codacy.com/project/badge/grade/1feffae1edbc424790eb86d330042d7e)](https://www.codacy.com/app/SuderPawel/hokuyo-python-lib)
[![Code Health](https://landscape.io/github/SuderPawel/hokuyo-python-lib/master/landscape.svg?style=flat)](https://landscape.io/github/SuderPawel/hokuyo-python-lib/master)

[![Code Climate](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib/badges/gpa.svg)](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib)
[![Test Coverage](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib/badges/coverage.svg)](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib/coverage)
[![Issue Count](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib/badges/issue_count.svg)](https://codeclimate.com/github/SuderPawel/hokuyo-python-lib)

This library provides simple implementation of SCIP protocol used in Hokuyo laser ranger finder in python.

Supported protocol is [SCIP 2.0](http://www.hokuyo-aut.jp/02sensor/07scanner/download/pdf/URG_SCIP20.pdf). This implementation was tested on [Hokuyo URG-04LX-UG01](https://www.hokuyo-aut.jp/02sensor/07scanner/urg_04lx_ug01.html).

How to use (PyPi)
-----------------------------

Simply. Add to `requirements.txt`

    -e git+git://github.com/SuderPawel/hokuyo-python-lib#egg=hokuyo-python-lib
