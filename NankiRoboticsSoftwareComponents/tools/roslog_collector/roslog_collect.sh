#!/bin/bash
cd ~/.ros/

DATE=`date "+%Y%m%d%I%M%S"`
echo $DATE
FILENAME=roslog_$DATE.zip
echo $FILENAME
zip -r ~/$FILENAME log
