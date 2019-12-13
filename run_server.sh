#!/usr/bin/env bash

cd /vdms
vdms.run 2> log.log

# If problems with initialization, try deleting db folder.
