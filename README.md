# UAS ROBOTIKA DAN SISTEM OTONOM
---

## NOMOR 1:
Sebuah robot bergerak dua roda (differential drive robot) telah memiliki jalur referensi hasil dari proses path planning (misalnya menggunakan Dijkstra atau A*). Jalur tersebut direpresentasikan sebagai serangkaian titik koordinat (𝑥,𝑦)  dari Titik Start (S) menuju Titik Goal (G). 

Pada pengujian awal, robot bergerak dengan pendekatan go-to-point sederhana, namun menunjukkan beberapa permasalahan berikut: 
- Robot sering menyimpang dari jalur referensi, terutama pada tikungan,
- Terjadi osilasi arah gerak saat robot mencoba kembali ke jalur,
- Error lintasan (cross-track error) cukup besar ketika kecepatan robot meningkat. 
 
Untuk mengatasi permasalahan tersebut, tim Anda diminta untuk merancang metode path tracking agar robot dapat mengikuti jalur referensi secara halus dan stabil, dengan mempertimbangkan keterbatasan kinematika robot differential drive. 
 
Pertanyaan:  
Bagaimana Anda merancang metode path tracking yang tepat agar robot bergerak dapat mengikuti jalur referensi dengan baik? 

Jelaskan secara rinci:
1. Prinsip kerja metode path tracking yang dipilih (misalnya Pure Pursuit, Stanley, atau metode berbasis error geometrik), termasuk konsep dasar yang digunakan untuk menentukan arah gerak robot.
2. Definisi dan peran error lintasan dalam path tracking, seperti cross-track error dan heading error, serta bagaimana error tersebut digunakan untuk mengoreksi arah gerak robot.
3. Pengaruh parameter path tracking (misalnya look-ahead distance pada Pure Pursuit) terhadap kestabilan dan kehalusan lintasan yang diikuti robot.
4. Analisis hasil simulasi, meliputi:
   - perbandingan antara jalur referensi dan lintasan aktual robot,
   - perilaku robot pada segmen lurus dan tikungan,
   - kondisi di mana path tracking gagal atau kurang optimal. 
 
Hasil dapat disajikan dalam bentuk simulasi Python dan visualisasi grafik (lintasan referensi vs lintasan robot, serta grafik error terhadap waktu).

---

# DIBUAT OLEH:
## KELOMPOK 1:
- JAROT WIWOHO               (41422110036)
- RAIHAN RAMANDHA SAPUTRA    (41422110039)
