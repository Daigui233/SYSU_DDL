#!/bin/bash
sleep 2
kill -9 4744 2>/dev/null
sleep 1
exec "/home/orangepi/Desktop/setupUI/dist/setup_webui"
