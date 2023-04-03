#!/usr/bin/env sh

echo "#ifndef LUREBOT_FIRMWARE_VERSION_H
#define LUREBOT_FIRMWARE_VERSION_H

#define LUREBOT_FIRMWARE_VERSION \"$(git describe --always --dirty --match 'NOT A TAG')\"

#endif
" > include/lurebot_firmware_version.h
