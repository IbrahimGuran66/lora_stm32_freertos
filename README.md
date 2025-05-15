# lora_stm32_freertos
/* ************************** ENDÜSTRİYEL ELEKTRİK ***************************** */
/* ************************ STM32 İLE LORA HABERLEŞME ************************** */
/* ************************* MÜHENDİS:İBRAHİM GÜRAN *************************** */
/* Not: Bu proje STM32 kartları ile denenmiş ve çalışmıştır. Verici kart yalnızca
 * "ping" verisini alıcıya gönderir. Alıcı ise "ping" standardında led yakar.
 * Verici kart NUCLEO F411, alıcı kart NUCLEO F103'tür ve board üzerinde pinoutları
 * farklıdır. Bu verinin gönderilmesi durumunda veya alınca vereceği
 *Farklı uygulamaları değiştirerek yapabilirsiniz. Kod üzerinde anlaşılır güç
 *noktaları da yorum satırlarıyla anlattım. Bu projede bana destek olan ve
 * bana bu proje üzerinde çalışmama izin veren başta sayın HİKMET ALKILINÇ ve YUSUF ÖZYER
 * olmak üzere ENDUSTRIYEL ELEKTRIK ARGE MERKEZİ mühendislerine ve doğrudan bağlantı
 *kurumasam da kütüphane oluşturmama yardım eden WARD ALMASARANI'ye teşekkür ederim.
 */
NOT: Bu projede sadece iki LoRa modülü kullanılmış, daha fazla lora modülü haberleşmesi henüz denenmemiştir.

 *KLASÖR İÇİNDEKİ DOSYALARI NEREDE KULLANIRIM? NE İŞE YAARLAR?

 * lora_conf.PNG : LoRa modüllerinin konfigürasyon ayarları. değiştirme ayarının tıpatıp aynısını çalıştırır.
 * RF_Setting(E22-E9X(SL))+V3.0.zip : LoRa konfigürasyon ayarı yapılan uygulama. EBYTE'in ülkedeki güncel sürümünü bulabilirsiniz.
 * lora_deneme_f103 : Projede kullanılan NUCLEO F103 kartına yüklenen kod. Yapmak istiyorsanız projeyi baştan kurmanızı tavsiye ederim.
 * lora_deneme_f411 : Projede kullanılan NUCLEO F411 kartına yüklenen kod.
 * şema.png : LoRa modülleri kartlara bu şekilde bu şekilde bağlanır.
 * E22-xxxT22S-V2.2_UserManual_EN_V1.5.pdf = Projede kullanılan LoRa modülünün kullanım kılavuzu.
 * NUCLEO-F411RE.pdf = Kulanılan boardların her birleştiği de kapsamlı kullanım kılavuzu.
 * WhatsApp Videosu 2025-04-02 14.35.45.mp4 = Proje açıklama videosu.
 * test.m4a = test videosu
 * LoRa-Haberlesme-Sistemi-Projesi.pptx = proje sunumu
 *mesafe_testi.mp4 = test videosu

NOT2: İki karttan da saat şeklinde maksimum hızda çalışmalar için maksimum miktarın sağlanması tavsiye edilir.
NOT3: İki kartın da aynı saat performansına sahip olması gerekmiyor.
