#define hallA_l 2
#define hallB_l 3
#define hallZ_l 4
#define hallA_r 6
#define hallB_r 7
#define hallZ_r 8

String binaryArray[6] = {"001", "011", "010", "110", "100", "101"};
int stateIndex_l = -1;  
int stateIndex_r = -1;
int count_l = 0;      
int count_r = 0;   
String previousState_l;  
String previousState_r; 

void setup() {
    pinMode(hallA_l, INPUT_PULLUP);
    pinMode(hallB_l, INPUT_PULLUP);
    pinMode(hallZ_l, INPUT_PULLUP);
    pinMode(hallA_r, INPUT_PULLUP);
    pinMode(hallB_r, INPUT_PULLUP);
    pinMode(hallZ_r, INPUT_PULLUP);
    Serial.begin(115200);
  
    int al_state = digitalRead(hallA_l);
    int bl_state = digitalRead(hallB)_l;
    int zl_state = digitalRead(hallZ_l);
    int ar_state = digitalRead(hallA_r);
    int br_state = digitalRead(hallB)_r;
    int zr_state = digitalRead(hallZ_r);
    
    String hall_state_l = String(al_state) + String(bl_state) + String(zl_state);
    String hall_state_r = String(ar_state) + String(br_state) + String(zr_state);
    
    previousState_l = hall_state_l;
    previousState_r = hall_state_r;

    // Tìm chỉ số tương ứng trong binaryArray
    for (int i = 0; i < 6; i++) {
        if (hall_state_l == binaryArray[i]) {
            stateIndex_l = i; 
            break; 
        }
    }
    for (int i = 0; i < 6; i++) {
        if (hall_state_r == binaryArray[i]) {
            stateIndex_r = i; 
            break; 
        }
    }
}

void loop() {
    int al_state = digitalRead(hallA_l);
    int bl_state = digitalRead(hallB)_l;
    int zl_state = digitalRead(hallZ_l);
    int ar_state = digitalRead(hallA_r);
    int br_state = digitalRead(hallB)_r;
    int zr_state = digitalRead(hallZ_r);
    
    String hall_state_l = String(al_state) + String(bl_state) + String(zl_state);
    String hall_state_r = String(ar_state) + String(br_state) + String(zr_state);

   
    // Kiểm tra nếu trạng thái Hall hiện tại khác với trạng thái trước đó
    if (hall_state_l != previousState_l) {
        // Tìm chỉ số của trạng thái Hall hiện tại
        for (int i = 0; i < 6; i++) {
            if (hall_state_l == binaryArray[i]) {
                // Xác định hướng quay
                if (i == (stateIndex_l + 1) % 6) {
                    count_l++;  // Tăng nếu trạng thái khớp với chỉ số tiếp theo
                    stateIndex_l = i; // Cập nhật stateIndex
                } else if (i == (stateIndex_l + 5) % 6) { // Quay ngược
                    count_l--; // Giảm nếu trạng thái khớp với chỉ số trước đó
                    stateIndex_l = i; // Cập nhật stateIndex
                }
                break; // Thoát vòng lặp khi tìm thấy trạng thái
            }
        };

        if (hall_state_r != previousState_r) {
        // Tìm chỉ số của trạng thái Hall hiện tại
        for (int i = 0; i < 6; i++) {
            if (hall_state_r == binaryArray[i]) {
                // Xác định hướng quay
                if (i == (stateIndex_r + 1) % 6) {
                    count_r++;  // Tăng nếu trạng thái khớp với chỉ số tiếp theo
                    stateIndex_r = i; // Cập nhật stateIndex
                } else if (i == (stateIndex_r + 5) % 6) { // Quay ngược
                    count_r--; // Giảm nếu trạng thái khớp với chỉ số trước đó
                    stateIndex_r = i; // Cập nhật stateIndex
                }
                break; // Thoát vòng lặp khi tìm thấy trạng thái
            }
        };

        // Cập nhật trạng thái trước đó
        previousState_l = hall_state_l;
        previousState_r = hall_state_r;
        /*Serial.print("HallA: "); Serial.print(a_state);
        Serial.print(" | HallB: "); Serial.print(b_state);
        Serial.print(" | HallZ: "); Serial.print(z_state);
        Serial.println();*/

        Serial.print(count_l);  // Encoder trái
        Serial.print(",");    
        Serial.println(count_r);
    }

    delay(0.5);  // Tạo delay nhỏ để tránh đọc quá nhanh
}
