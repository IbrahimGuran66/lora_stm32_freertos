/* **************************ENDUSTRIYEL ELEKTRIK***************************** */
/* ************************STM32 İLE LORA HABERLEŞME************************** */
/* *************************ENGINEER:IBRAHIM GURAN**************************** */
/* Not: Bu proje STM32 kartları ile denenmiş ve çalışmıştır. Verici kart yalnızca
 * "ping" verisini alıcıya gönderir. Alıcı ise "ping" komutunu aldığında led yakar.
 * Verici kart NUCLEO F411, alıcı kart NUCLEO F103'tür ve board üzerinde pinoutları
 * aynıdır. Bu veri gönderme işlemini değiştirebilir veya alınca vereceği cevabı
 * değiştirerek farklı uygulamalar yapabilirsiniz. Kod üzerinde anlaşılması güç
 * noktaları da yorum satırlarıyla anlattım. Bu projede bana destek olan ve
 * bana bu proje üzerinde çalışmama izin veren başta sayın HIKMET ALKILINÇ ve YUSUF OZYER
 * olmak üzere ENDUSTRIYEL ELEKTRIK ARGE MERKEZİ mühendislerine ve doğrudan bağlantı
 * kuramasam da kütüphane oluşturmama yardım eden WARD ALMASARANI'ye teşekkür ederim.
 */
NOT: Bu projede sadece iki LoRa modülü kullanılmıştır, daha fazla lora modülü haberleşmesi henüz denenmemiştir.

 * KLASÖR İÇİNDEKİ DOSYALARI NEREDE KULLANIRIM? NE İŞE YARARLAR?

 * lora_conf.PNG : LoRa modüllerinin konfigürasyon ayarları. Buradaki ayarın tıpatıp aynısını yaparsanız çalışır.
 * RF_Setting(E22-E9X(SL))+V3.0.zip : LoRa konfigürasyon ayarı yapılan uygulama. EBYTE'ın sitesinden güncel versiyonunu bulabilirsiniz.
 * lora_deneme_f103 : Projede kullanılan NUCLEO F103 kartına yüklenen kod. Yapmak istiyorsanız projeyi baştan kurmanızı tavsiye ederim.
 * lora_deneme_f411 : Projede kullanılan NUCLEO F411 kartına yüklenen kod.
 * şema.png : LoRa modülleri kartlara bu modda bu şekilde bağlanır.
 * E22-xxxT22S-V2.2_UserManual_EN_V1.5.pdf = Projede kullanılan LoRa modülünün kullanma kılavuzu.
 * NUCLEO-F411RE.pdf = Kulanılan boardların her ikisini de kapsayan kullanım kılavuzu.
 * WhatsApp Video 2025-04-02 at 14.35.45.mp4 = Proje açıklama videosu.
 * test.m4a = test videosu
 * LoRa-Haberlesme-Sistemi-Projesi.pptx = proje sunumu
 * mesafe_testi.mp4 = test videosu

NOT2:  İki kartın da clock konfigürasyonunda max hızda çalışmaları için frekansı max yapmanızı tavsiye ederiz. 
NOT3: İki kartın da aynı clock frekansına sahip olmasına gerek yok.
