# robot
📦 2. Clone โปรเจกต์จาก GitHub

⚠️ ถ้า repo ของเคนตะแก้ submodule ออกแล้ว (เป็นโฟลเดอร์ปกติ) → ใช้บรรทัดแรก
แต่ถ้ายังมี submodule อยู่ → ใช้แบบที่มี --recurse-submodules

✅ แบบปกติ
git clone https://github.com/SopitaJW/rbkairos_ws.git

✅ ถ้ามี submodule (เข้าโฟลเดอร์ไม่ได้ตอนดูบน GitHub)
git clone --recurse-submodules https://github.com/SopitaJW/rbkairos_ws.git


เข้าไปในโฟลเดอร์โปรเจกต์:

cd rbkairos_ws

⚙️ 3. Build workspace
colcon build


หลังจาก build เสร็จ ให้ source environment:

source install/setup.bash

🚀 4. การรัน Launch File

เช่นถ้ามี launch ชื่อ rbkairos_mecanum.launch.py
ให้สั่ง:

ros2 launch robot_model rbkairos_mecanum.launch.py


💡 ตรวจสอบชื่อ package และไฟล์ launch ให้ตรงกับที่มีใน src/

🧠 5. โครงสร้างไฟล์ (ตัวอย่าง)
rbkairos_ws/
├── src/
│   ├── robotnik_description/
│   ├── robotnik_sensors/
│   ├── ros2_diff_drive_robot/
│   └── robot_model/
├── build/
├── install/
├── log/
├── CMakeLists.txt
└── README.md

🧹 6. ถ้าแก้โค้ดแล้ว build ใหม่
colcon build --symlink-install
source install/setup.bash
