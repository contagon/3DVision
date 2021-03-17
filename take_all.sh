declare -a arr=(S L R)
DIR=images
for i in "${arr[@]}"
do
    echo "Starting $i"
    python take_calib_images.py -l 2 -r 4 --start_wait 5 -o ${DIR}/$i
done

# Remove off images
rm ${DIR}/L/R_*
rm ${DIR}/R/L_*

# Calibrate
python calibrate.py -s ${DIR}/S -l ${DIR}/L -r ${DIR}/R -w 9 -t 7 -o ${DIR}/params.npz -d 0.9375