ip=192.168.0.157

scp ./build/component/VMW-esx-7.0.1-thgpio-*.zip root@${ip}:/scratch/downloads/

ssh root@${ip} '/scratch/downloads/install_thgpio.sh'
