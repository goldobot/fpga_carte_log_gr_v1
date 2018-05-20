#!/bin/bash

/bin/rm -rf src/version.vhd.body
/bin/rm -rf src/version.vhd.tmp
/bin/rm -rf src/version.vhd
/usr/bin/git describe --abbrev=8 --dirty --always --tags 2>/dev/null >src/version.vhd.tmp
/bin/dd if=src/version.vhd.tmp of=src/version.vhd.body bs=1 count=8
/bin/cat src/version.vhd.head src/version.vhd.body src/version.vhd.tail > src/version.vhd


