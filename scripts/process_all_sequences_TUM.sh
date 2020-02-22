#!/bin/bash

DIR_PATH=$1
DEBUG_LOG=process_all_debug.log
PROCESSED_FILES_LOG=processed_files_order.log

# Clear previous logs if exist
rm $DEBUG_LOG
rm $PROCESSED_FILES_LOG

for file in "$DIR_PATH"/*
do
  echo Proccessing file: $file on `date` >> $PROCESSED_FILES_LOG
  ./process_one_TUM_sequence.sh $file >> $DEBUG_LOG 2>&1
done






