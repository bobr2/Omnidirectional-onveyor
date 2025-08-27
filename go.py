import socket
import struct
import time
import numpy
# === Настройки ===
RECV_PORT = 5005
id_to_port = {
    1: ["192.168.88.223",4311],
    2: ["192.168.88.222",4312],
    3: ["192.168.88.221",4313],
    4: ["192.168.88.220",4314],
    5: ["192.168.88.219",4315],
    6: ["192.168.88.218",4316],
    7: ["192.168.88.217",4317],
    8: ["192.168.88.216",4318],
    9: ["192.168.88.215",4319],
}

def create_packet(board_id, lt, rt, lb, rb):
    def to_signed_16(val):
        if not -255 <= val <= 255:
            raise ValueError("Value out of range (-255 to 255): {}".format(val))
        return val & 0xFFFF  # 16 битное представление

    packet = bytearray(12)
    packet[0] = 0xAA  
    packet[1] = board_id & 0xFF
    packet[2] = 0x02  # Тип операции

    # Преобразование значений в 16-битные signed
    values = [lt, rt, lb, rb]
    for i, val in enumerate(values):
        signed_val = to_signed_16(val)
        packet[3 + i * 2] = (signed_val >> 8) & 0xFF
        packet[4 + i * 2] = signed_val & 0xFF


    crc = 0
    for i in range(1, 11):
        crc ^= packet[i]
    packet[11] = crc

    return packet


START_BYTE = 0xAA
GROUP_SIZE = 4  # 4 числа на каждый ID
NUM_IDS = 9     # 36 чисел / 4 = 9 ID
PACKET_SIZE = 12  # AA ID TYPE D1 D2 D3 D4 CRC

# === Сокеты ===
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("", RECV_PORT))
print(f"🟢 Слушаю UDP порт {RECV_PORT} для входящих пакетов...")

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:

        data, addr = recv_sock.recvfrom(144)
        """
        if len(data) == 72:
            # Распаковка для читаемости
            motors = struct.unpack('<36h', data)
            print(f"[IN ] Управление от ПК: {motors}")
            
            print(f"[OUT] Отправлено моторной Arduino: {motors}")
        else:
            print(f"Неверный размер пакета: {len(data)} байт")
        """
        # Распаковываем 36 int16 из байтов
        values = list(struct.unpack('<36h', data))  # little-endian int16

        print(f"📥 Получено 36 значений от {addr[0]}")

        # Обрабатываем каждую группу из 4 чисел
        for i in range(9):
            ID = i + 1
            chunk = values[i * GROUP_SIZE:(i + 1) * GROUP_SIZE]
            LT, RT, LB, RB = chunk
            send_sock.sendto(create_packet(1, LT, RT, LB, RB), (id_to_port[i+1][0], id_to_port[i+1][1]))
            print(LT, RT, LB, RB, i+1)
            print("sent")
        

except KeyboardInterrupt:
    print("\n🛑 Прервано пользователем (Ctrl+C). Завершение...")
finally:
    for i in range(9):
            ID = i + 1
            chunk = values[i * GROUP_SIZE:(i + 1) * GROUP_SIZE]
            LT, RT, LB, RB = chunk
            send_sock.sendto(create_packet(1, LT, RT, LB, RB), (id_to_port[i+1][0], id_to_port[i+1][1]))
            print(0, 0, 0, 0, i+1)
            print("sent")
    recv_sock.close()
    send_sock.close()
    print("🔌 Сокеты закрыты. Скрипт завершён.")