# WSN
# Tổng Quan Về Mạng Cảm Biến Không Dây

## 1.1 Đặt Vấn Đề và Tổng Quan
Trong nông nghiệp hiện đại, đặc biệt là nông nghiệp thông minh, việc giám sát và quản lý điều kiện môi trường, như nhiệt độ, đóng vai trò quan trọng đối với sự phát triển của cây trồng. Nhiệt độ ảnh hưởng trực tiếp đến quá trình sinh trưởng, ra hoa, kết trái và năng suất cây trồng.

Nhà kính giúp cung cấp các thông số môi trường lý tưởng, với năng suất tăng 5-8 lần so với phương pháp truyền thống. Tuy nhiên, mỗi loại cây trồng có một khoảng nhiệt độ tối ưu khác nhau, ví dụ:
- **Cà chua**: Tối ưu 29°C, tối thiểu 10°C, tối đa 35°C.
- **Dưa chuột**: Tối ưu 35°C.
- **Hoa đồng tiền**: Tối ưu từ 12°C đến 34°C.

Để giải quyết vấn đề này, đồ án tập trung vào việc **Thiết Kế Và Triển Khai Hệ Thống Đo Nhiệt Độ Không Dây** sử dụng công nghệ Bluetooth Low Energy (BLE). Hệ thống hỗ trợ giám sát nhiệt độ chính xác, giúp người nông dân điều chỉnh môi trường kịp thời nhằm tối ưu hóa sự phát triển cây trồng.

## 1.1.1 Mục Tiêu và Mục Đích
### Mục tiêu:
- Xây dựng các node cảm biến để giám sát nhiệt độ cây trồng.
- Độ phân giải hiển thị nhiệt độ: **0.1°C**.
- Sử dụng pin Li-ion có thể sạc lại.
- Thời gian lấy mẫu: **20 giây** (có thể điều chỉnh).
- Phạm vi hoạt động: **1000m²**.
- Tích hợp nút bắt đầu đo và đèn báo trạng thái.
- Hỗ trợ quản lý **tối thiểu 7 thiết bị đo**.
- Xây dựng phần mềm và giao diện web để:
  - Thu thập dữ liệu đo.
  - Quản lý dữ liệu.
  - Xuất báo cáo dạng Excel.

### Mục đích:
Thiết kế và triển khai hệ thống giám sát nhiệt độ không dây bằng công nghệ BLE cho nông nghiệp, đặc biệt là nhà kính và vườn rau. Hệ thống hỗ trợ:
- Giám sát chính xác nhiệt độ môi trường.
- Điều chỉnh điều kiện tối ưu cho cây trồng.
- Tăng năng suất, chất lượng cây trồng.
- Tiết kiệm năng lượng, chi phí lắp đặt và bảo trì.
- Hỗ trợ phát triển nông nghiệp bền vững.

## 1.1.2 Phạm Vi Áp Dụng
- Giám sát nhiệt độ trong nhà kính, vườn rau, hoặc khu vực trồng trọt cần kiểm soát môi trường.
- Hệ thống BLE linh hoạt, dễ dàng mở rộng để áp dụng cho:
  - Các trang trại công nghệ cao.
  - Khu vực trồng trọt lớn.
  - Nghiên cứu nông nghiệp.
  - Các khu vực canh tác ngoài trời hoặc nhà lưới cần kiểm soát tự động.

## 1.1.3 Nội Dung Đồ Án
### **Chương 1: Tổng Quan**
- Trình bày vấn đề cần giải quyết, mục tiêu và phạm vi của đồ án.
- Kiến thức tổng quan về mạng cảm biến không dây:
  - Cấu trúc mạng.
  - Giao thức phổ biến.
  - Yêu cầu thiết kế hệ thống đo nhiệt độ không dây.

### **Chương 2: Thiết Kế Hệ Thống**
- Các giải pháp và phương án thiết kế hệ thống:
  - Lựa chọn cấu trúc mạng và công nghệ truyền không dây.
  - Thiết kế phần cứng (sơ đồ khối, linh kiện, mạch in).
  - Thiết kế phần mềm.

### **Chương 3: Triển Khai và Đánh Giá**
- Quá trình thử nghiệm hệ thống thực tế.
- Kết quả đạt được và đánh giá hiệu quả.
- Phân tích điểm mạnh, hạn chế và đề xuất hướng phát triển tương lai.
