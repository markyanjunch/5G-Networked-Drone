#i/bin/bash
echo 'date' >> /home/dji/Desktop/output1.txt
echo "Automatically open VE450..."
cd /home/dji/Desktop/OSDK/Onboard-SDK-3.9.0/build/bin/
./VE450 UserConfig.txt
exit 0
