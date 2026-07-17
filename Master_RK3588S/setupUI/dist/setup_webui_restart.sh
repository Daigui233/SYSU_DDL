#!/bin/bash
sleep 2
kill -9 4332 2>/dev/null
sleep 1
exec "/home/orangepi/Desktop/SYSU_DDL/Master_RK3588S/setupUI/dist/setup_webui"
