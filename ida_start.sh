#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}##############################################################${NC}"
echo -e "${GREEN}###   KAAN TEKNOLOJİ KULÜBÜ - ERTUĞRUL İDA BAŞLATILIYOR   ####${NC}"
echo -e "${GREEN}##############################################################${NC}"

PROJECT_DIR="/home/kaan/kulup_ws/KAAN-Teknoloji/Ertugrul/Teknofest26"

echo -e "\n${YELLOW}[1/6] Performance and Permissions...${NC}"
sudo jetson_clocks --fan > /dev/null 2>&1
sudo nvpmodel -m 0 > /dev/null 2>&1
xhost +local:docker > /dev/null 2>&1

sudo chmod 777 /dev/ttyUSB* > /dev/null 2>&1
sudo chmod 777 /dev/ttyACM* > /dev/null 2>&1
sudo chmod 777 /dev/video* > /dev/null 2>&1

echo -e "\n${YELLOW}[2/6] Network Settings...${NC}"
CURRENT_IP=$(hostname -I | awk '{print $2}')
if [ -z "$CURRENT_IP" ]; then CURRENT_IP='127.0.0.1'; fi

if [ -f "$PROJECT_DIR/communication/fastdds_template.xml" ]; then
    sed "s/IP_ADRESS_PLACEHOLDER/$CURRENT_IP/g" "$PROJECT_DIR/communication/fastdds_template.xml" > "$PROJECT_DIR/communication/fastdds_lowmem.xml"
fi

echo -e "\n${YELLOW}[3/6] Dynamic ACM port detection (Cubepilot)...${NC}"

ACM_PORTS=($(ls /dev/ttyACM* 2>/dev/null | sort -V))
PORT_COUNT=${#ACM_PORTS[@]}

if [ $PORT_COUNT -eq 0 ]; then
    echo -e "${RED}[ERROR] Cubepilot is not connect! No ACM ports found!${NC}"
    exit 1
elif [ $PORT_COUNT -eq 1 ]; then
    DDS_PORT="${ACM_PORTS[0]}"
    MAVLINK_PORT="${ACM_PORTS[0]}"
    echo -e "${YELLOW}[WARNING] Only one port detected: $DDS_PORT${NC}"
else
    DDS_PORT="${ACM_PORTS[$((PORT_COUNT-1))]}"
    MAVLINK_PORT="${ACM_PORTS[$((PORT_COUNT-2))]}"
fi

echo -e "${GREEN}[OK] Detected DDS port (Highest): $DDS_PORT${NC}"
echo -e "${GREEN}[OK] Detected MAVLink port (Lowest): $MAVLINK_PORT${NC}"

echo "$MAVLINK_PORT" > /tmp/mavlink_port.txt

echo -e "\n${YELLOW}[4/6] Cubepilot rebooting with MAVLink ($MAVLINK_PORT)...${NC}"
python3 "$PROJECT_DIR/utils/reboot_pixhawk.py" "$MAVLINK_PORT"

echo -e "${YELLOW}[INFO] Detecting ports after reboot...${NC}"
sleep 3
ACM_PORTS_AFTER=($(ls /dev/ttyACM* 2>/dev/null | sort -V))
PORT_COUNT=${#ACM_PORTS_AFTER[@]}

if [ $PORT_COUNT -eq 0 ]; then
    echo -e "${RED}[ERROR] Cubepilot is not connect! No ACM ports found!${NC}"
    exit 1
elif [ $PORT_COUNT -eq 1 ]; then
    DDS_PORT="${ACM_PORTS_AFTER[0]}"
    MAVLINK_PORT="${ACM_PORTS_AFTER[0]}"
    echo -e "${YELLOW}[UYARI] Tek port tespit edildi: $DDS_PORT${NC}"
else
    DDS_PORT="${ACM_PORTS_AFTER[$((PORT_COUNT-1))]}"
    MAVLINK_PORT="${ACM_PORTS_AFTER[$((PORT_COUNT-2))]}"
fi

echo -e "\n${YELLOW}[5/6] Micro-ROS Agent ($DDS_PORT) Başlatılıyor...${NC}"
docker stop micro_ros_agent > /dev/null 2>&1
docker run -d --rm --net=host --privileged \
    --name micro_ros_agent \
    -v /dev:/dev \
    -v "$PROJECT_DIR":/root/ida_ws \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/ida_ws/communication/fastdds_lowmem.xml \
    microros/micro-ros-agent:humble \
    serial --dev "$DDS_PORT" baudrate=921600

echo -e "\n${YELLOW}[6/6] Main container is starting...${NC}"
docker stop ida_main_control > /dev/null 2>&1

echo -e "${GREEN}[COMPELETED]...${NC}"

docker run -d --rm \
    --name ida_main_control \
    --runtime nvidia \
    --net=host \
    --privileged \
    -v /home/kaan/kulup_ws/KAAN-Teknoloji/Ertugrul/Teknofest26:/root/ida_ws \
    -e LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1:/lib/aarch64-linux-gnu/libGLdispatch.so.0" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/ida_ws/communication/fastdds_lowmem.xml \
    -e PYTHONUNBUFFERED=1 \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /tmp:/tmp \
    --device /dev/nvhost-nvenc1:/dev/nvhost-nvenc1 \
    --device /dev/nvhost-nvdec:/dev/nvhost-nvdec \
    --device /dev/gpiochip0:/dev/gpiochip0 \
    --device /dev/gpiochip0:/dev/gpiochip0 \
    --group-add $(cut -d: -f3 < <(getent group gpio)) \
    --shm-size=8g \
    --ulimit memlock=-1 \
    --ulimit stack=67108864 \
    ida_tf_image_l4 \
    tail -f /dev/null

echo -e "${GREEN}[COMPELETED] Entering inside the container.${NC}"

docker exec -it ida_main_control bash

echo -e "\n${YELLOW}Bağlantı kapatıldı. Arka plandaki konteyner durduruluyor...${NC}"
docker stop ida_main_control > /dev/null 2>&1
echo -e "${GREEN}Konteyner başarıyla kapatıldı. İyi çalışmalar!${NC}"