#!/bin/sh
python ./db_gen/generate_face_db.py --db db --db-filename embeddings --include-path include $@
