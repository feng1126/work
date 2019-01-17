#!/bin/sh


echo -e "--------------------------------- Start----------------------------------" 
path=$(cd $(dirname $0); pwd)
work_path=$(ls -a |grep BF_IMAGE)
for work in  $work_path
do 
   echo $work_path
   rm -rf ./$work/cam0
   rm -rf ./$work/cam1
   unzip -d ./$work/cam0 "$path/$work/CAM-L/*zip" 
   unzip -d ./$work/cam1 "$path/$work/CAM-R/*zip"  
done
echo -e "--------------------------------- Over------------------------------------" 
