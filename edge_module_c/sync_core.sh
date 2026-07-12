#!/bin/sh
# core/ → esp32/edge_alimi/ 사본 동기화 (단일 진실 유지)
cp core/*.c core/*.h esp32/edge_alimi/
echo "synced: $(ls core/*.c core/*.h | wc -l) files"
