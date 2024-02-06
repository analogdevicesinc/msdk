#!/bin/sh
python ./db_gen/generate_face_db.py --db db --base include/baseaddr.h $@
