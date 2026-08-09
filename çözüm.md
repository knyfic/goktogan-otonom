**oneri30**:
RRPi tarafında finish_pickup komutu gelince stepper thread'ini beklemek yerine, stepper zaten bitmiş olacak çünkü PAYLOAD_SERVO_WAIT_S hesabı aşağı hareketin süresini kapsamalı. Hesap:
17143 adım × 0.001s/adım × 2 (HIGH+LOW) = 34.3 saniye
PAYLOAD_SERVO_WAIT_S = 35 saniye yap 
