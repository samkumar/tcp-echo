#!/bin/bash

JLinkExe -device atsamr21e18 -speed 100 -if swd setnvm.jlink
