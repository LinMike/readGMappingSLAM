#! /bin/bash

link_name=(opencv_dep_cudart \
 opencv_dep_nppc \
 opencv_dep_nppc \
 opencv_dep_nppial \
 opencv_dep_nppicc \
 opencv_dep_nppicom \
 opencv_dep_nppidei \
 opencv_dep_nppif \
 opencv_dep_nppig \
 opencv_dep_nppim \
 opencv_dep_nppist \
 opencv_dep_nppisu \
 opencv_dep_nppitc \
 opencv_dep_npps \
 opencv_dep_cublas \
 opencv_dep_cufft)

cuda_dir=/usr/local/cuda/targets/x86_64-linux/lib
local_dir=/usr/local/lib
prifix=lib
ext=.so

echo last link name = ${link_name[-1]}
# echo all = ${link_name[*]}

for VAR in ${link_name[*]}
do
    temp_arr=(`echo $VAR | tr '_' ' '`)
    # echo ${temp_arr[*]}
    target_name=${local_dir}/${prifix}$VAR$ext
    source_name=${cuda_dir}/${prifix}${temp_arr[-1]}${ext}
    echo $target_name
    echo $source_name
    echo "create soft link for " $target_name
    if test -e $source_name
    then
        echo "run sudo ln -s $source_name $target_name"
        sudo ln -s $source_name $target_name
    else
        echo 'source file not exist'
    fi
done

# sudo ln -s /usr/local/cuda/targets/x86_64-linux/lib/libcudart.so /usr/local/lib/libopencv_dep_cudart.so

