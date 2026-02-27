Prupose of this branch: This branch contains updates needed to deploy MySensors nodes that I am currentyl using. I particular, it has updates for NRF52810 and WIO-E5 (STM32WL) based nodes.
- Includes Mysensors STM32 archictecture hal for STM32 cube support.  This version is based on KoolRU STM32 MySensors support, and is yet completely aligned wit MySensors development branch.
- Adds RLSX126X to the transport hal.  This driver uses the RadioLib library for low level programming to support the STM32WL series
- Adds CPU voltage and Temperature support to the STM32 architecture hal

MySensors Library Info
-------------
MySensors Library v2.4.0-alpha

Please visit www.mysensors.org for more information

Current version in Arduino IDE [![arduino-library-badge](https://www.ardu-badge.com/badge/MySensors.svg)](https://www.ardu-badge.com/MySensors)

Documentation
-------------
[master](https://www.mysensors.org/apidocs/index.html) [development](https://www.mysensors.org/apidocs-beta/index.html)

CI statuses
-----------
Current build status of master branch: [![Build Status](https://ci.mysensors.org/job/MySensors/job/MySensors/job/master/badge/icon)](https://ci.mysensors.org/job/MySensors/job/MySensors/job/master/)

Current build status of development branch: [![Build Status](https://ci.mysensors.org/job/MySensors/job/MySensors/job/development/badge/icon)](https://ci.mysensors.org/job/MySensors/job/MySensors/job/development/)

Current build status of master branch (nightly build of Arduino IDE): [![Build Status](https://ci.mysensors.org/job/MySensors-nightly-IDE/job/MySensors/job/master/badge/icon)](https://ci.mysensors.org/job/MySensors-nightly-IDE/job/MySensors/job/master/)

Current build status of development branch (nightly build of Arduino IDE): [![Build Status](https://ci.mysensors.org/job/MySensors-nightly-IDE/job/MySensors/job/development/badge/icon)](https://ci.mysensors.org/job/MySensors-nightly-IDE/job/MySensors/job/development/)