İHA Gazebo Simülasyonu – Otonom Görüntü İşleme ve Yük Görevi Teknik Gereksinim Dokümanı
1. Proje ve Simülasyon Ortamı
Bu proje, bir quadcopter İHA’nın Gazebo simülasyonu içerisinde kamera tabanlı görüntü işleme kullanarak belirli geometrik hedefleri tespit etmesini, hedeflere uygun yükleri bırakmasını, daha sonra ilk hedefte bırakılan yükü tekrar yerden almasını ve görev sonunda kalkış noktasına geri dönmesini amaçlamaktadır.
Simülasyon ortamı
kod moduler bir sekilde yazılmalıdır bu sekilde kodun hatalı calısması durumunda iligli dosya daha kolay bir sekilde duzenlenebilir. ornegin goruntu isleme baska bir dosya halinde vb vb.
	Simulator: Gazebo Sim
	Gazebo sürümü: 10.5.0
	Gazebo dağıtımı: gz-jetty
	ArduPilot: ardupilot_gazebo + ArduPilot SITL
	Frame: Quadcopter
	Frame Type: 1
	Class: 1
	İşletim sistemi: Ubuntu 24.04
  kullanılacak olan dil: python

2. Yarışma Alanı
Simülasyondaki yarışma alanının boyutları:
	Genişlik: 30 metre
	Uzunluk: 100 metre
	Toplam alan: 3000 m²
İHA’nın göreve başlayacağı konum önceden bilinmemektedir.
Hedefler bu 30 × 100 metrelik alan içerisinde rastgele konumlara yerleştirilecektir.
Sistemin hedeflerin konumlarını önceden bilmesine izin verilmemelidir. Hedef konumları yalnızca kamera ve görüntü işleme sistemi kullanılarak tespit edilmelidir.

3. Tespit Edilecek Hedefler
Sistemin görüntü işleme ile iki farklı geometrik hedefi ayırt etmesi gerekmektedir.
Hedef 1 – Mavi Altıgen
	Renk: Mavi
	Şekil: Altıgen
	Bir kenar uzunluğu: 2 metre
Hedef 2 – Kırmızı Üçgen
	Renk: Kırmızı
	Şekil: Üçgen
	Bir kenar uzunluğu: 1 metre
Renk ve geometrik şekil birlikte değerlendirilmelidir.
Örneğin:
Mavi + Altıgen  → Altıgen hedef
Kırmızı + Üçgen → Üçgen hedef
Görüntü işleme sistemi yalnızca şekle veya yalnızca renge güvenmemelidir.

4. Kamera Konumu
İHA’nın kamerası aşağıya, yani zemine dik olacak şekilde konumlandırılmıştır.
Kamera:
	İHA’nın tam altında bulunur.
	Görüntü işleme temel olarak downward-facing görüntü üzerinden yapılacaktır.
Gazebo’daki kamera görüntü topic’i:
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
Kod içerisinde topic adı hard-code edilmek yerine mümkünse başlangıçta bir configuration variable olarak tanımlanmalıdır. bunun temel sebebi kodun yalnızca simuluasyon iicn degilde basit parameter degisimleri ile gercek quadcopter ile kullanıma hazır hale getirilmesini kolaylastırmaktır.
Örneğin:
CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

5. Kodlama Kuralları
Kod, doğrudan tek bir uzun script şeklinde yazılmamalıdır.
Kod okunabilir, modüler ve tekrar kullanılabilir olmalıdır.
Özellikle tekrar tekrar kullanılacak parametreler kodun başında veya ayrı bir configuration bölümünde tanımlanmalıdır.
Örneğin:
TAKEOFF_ALTITUDE = 10.0
TARGET_APPROACH_ALTITUDE = 1.0

SEARCH_TIMEOUT = ...
MAX_PICKUP_ATTEMPTS = 5

FIELD_WIDTH = 30.0
FIELD_LENGTH = 100.0

HEXAGON_SIDE_LENGTH = 2.0
TRIANGLE_SIDE_LENGTH = 1.0
Benzer şekilde:
	kamera topic’i
	hedef renkleri
	hedef geometrileri
	maksimum deneme sayısı
	kalkış irtifası
	yaklaşma irtifası
	tarama parametreleri
	güvenlik mesafeleri
	hız parametreleri
vb vb...
tek bir yerde tanımlanmalıdır.
Magic number kullanılmamalıdır.

6. Yük Bırakma Mekanizması
Yük bırakma mekanizması Gazebo modelinde fiziksel olarak ayrıntılı şekilde modellenmemiştir
Bunun temel sebebi mekanik mekanizmanın simülasyon karmaşıklığını gereksiz şekilde artırmasıdır.
Yazılım tarafında yük bırakma ve alma mekanizması için gerekli fonksiyonlar dummy/mock implementation olarak hazırlanmalıdır.
Örneğin:
def release_payload(payload_type):
    """
    Gazebo'da gerçek servo mekanizması modellenmediği için
    şimdilik dummy implementation.
    Gerçek sistemde burada PWM/servo komutu gönderilecektir.
    """
    pass
ve:
def pickup_payload(payload_type):
    """
    Gazebo simülasyonunda fiziksel pickup mekanizması modellenmedi.
    Gerçek sistemde servo/mekanik mekanizma burada kontrol edilecektir.
    wait(3s)
    """
    pass
Ancak bu fonksiyonlar tamamen boş ve anlamsız olmamalıdır.
Fonksiyon çağrıları, durum değişimleri, başarılı/başarısız sonuçlar ve gerekli loglama sistem içerisinde bulunmalıdır.
İleride gerçek donanıma geçildiğinde yalnızca bu fonksiyonların içeriği değiştirilerek sistemin geri kalanının kullanılabilmesi hedeflenmelidir.

7. Kritik Güvenlik Kuralı
Bu kural kesinlikle uygulanmalıdır:
İHA, Görev tamamlanana kadar hiçbir koşulda otomatik olarak DISARM edilmemelidir.
Görev sırasında:
	hedef bulunamaması,
	yük bırakılamaması,
	yük alınamaması,
	görüntü işleme hatası,
	pickup başarısızlığı,
	görev başarısızlığı
ihanın yere deymesi(tasarlalan yuk mekanizması sebebiyle ihanın yuku yerden alması icin disarm olmadan yere inis yapıp yuku alması gerekmektedir)
gibi durumlar İHA’nın disarm edilmesine neden olmamalıdır.
Görev başarısız olsa bile sistem:
Görev başarısız
        ↓
Return To Launch
        ↓
RTL tamamlanması
        ↓
Görev 3
şeklinde ilerlemelidir.
Görev 3 tamamlanmadan disarm yapılmamalıdır.

8. Genel Görev Akışı
Görev üç ana bölümden oluşmaktadır:
Görev 1
İki farklı yükün ilgili hedef alanlarına bırakılması.
Görev 2
İlk hedef alanına daha önce bırakılmış olan yükün tekrar bulunması ve yerden alınması.
Görev 3
Return To Launch (RTL) gerçekleştirilmesi.

9. Görev 1 – Hedefleri Bulma ve Yükleri Bırakma
9.1 Takeoff
simulasyonu kamera pov ile izlemek icin ilgili pencere acılır cv2 show vb..
İHA mevcut başlangıç konumundan:
10 metre
irtifaya kalkış yapar.
Başlangıç konumu önceden bilinmediği için sistem hedefleri aramaya bu noktadan başlamalıdır.

9.2 İlk Hedefi Arama
Takeoff tamamlandıktan sonra kamera görüntüsü sürekli alınır.
Görüntü işleme sistemi:
	mavi altıgeni
	kırmızı üçgeni
aramalıdır.
Öncelikle İHA’nın bulunduğu mevcut bölge belirli bir süre boyunca taranabilir.
Bu süre içerisinde hedef bulunursa:
Hedef bulundu
    ↓
Hedefin renk + şekil bilgisi belirlenir
    ↓
Hedef kayıt altına alınır(ilk hdef bulunda bulunan ilk hedef veriablı= mavi altıgen) vb.

10. Hedef Bilgisinin Kaydedilmesi
Bir hedef tespit edildiğinde hedef yalnızca anlık olarak kullanılmamalıdır.
Hedef bilgisi sistem içerisinde kayıt altına alınmalıdır.
Minimum kayıt:
target_id
target_type
target_color
gps_latitude
gps_longitude
detection_confidence
Örneğin:
Target:
    type: HEXAGON
    color: BLUE
    latitude: ...
    longitude: ...
Bu gereklidir çünkü aynı bölgede birden fazla geometrik şekil bulunabilir.
Sistem daha sonra hangi hedefi ilk tespit ettiğini ve hangi hedefin henüz işlenmediğini bilmelidir.
birden fazla hedefi aynı anda gordugu zaman karısıklık yasamamalıdır. sıralı bir sekilde isini yapması gerekmektedir.

11. Hedef Bulunamazsa – Alan Tarama
İHA bulunduğu konumda belirlenen süre içerisinde hedef tespit edemezse rastgele uçuş yapmamalıdır.
Bunun yerine sistematik bir search pattern / coverage path planning algoritması kullanılmalıdır.
Bu amaçla alan tarama algoritması seçilirken:
	toplam uçuş mesafesi
	tarama sırasında kamera görüş alanının kapsanması
	tekrar ziyaret edilen alan miktarı
	dönüş sayısı
	uçuş süresi
dikkate alınmalıdır.
Özellikle lawnmower / boustrophedon coverage path benzeri bir tarama algoritması uygun adaylardan biridir.
Amaç:
30 × 100 m alanı mümkün olduğunca az uçuş mesafesi ve minimum tekrar ile tamamen taramak.

12. Hedefe Yaklaşma Algoritması
Hedef tespit edildiğinde doğrudan hassas bir şekilde hedef merkezine inmeye çalışılmamalıdır.
Batarya ve zaman optimizasyonu açısından kademeli yaklaşım kullanılmalıdır.
Önerilen yaklaşım:
10m veya üstü bir irtifada ise
    ↓
Kaba yatay yaklaşma
    ↓
Hedef bölgesine yaklaşma
    ↓
Daha düşük irtifaya kademeli iniş
    ↓
Görüntü işleme ile hassas merkezleme
    ↓
1 m hedef irtifası
İHA 10 metre gibi yüksek bir irtifada hedefin tam merkezine milimetrik hassasiyetle uçmaya çalışmamalıdır.
Önce hedefin bulunduğu bölgeye kaba şekilde yaklaşmalıdır.
Hedef bölgesine yaklaşıldığında irtifa azaltılmalı ve görüntü çözünürlüğü/tespit doğruluğu artırılmalıdır.
fakat yukun bırakmadan veya yuku almadan once tam orta oldugundan emin olmalıdır.

13. Kritik Yaklaşma Problemi
İHA alçaldıkça hedef kameranın görüntüsünde büyüyecektir.
Ancak çok fazla alçalındığında hedefin tamamı kamera görüntüsünden çıkabilir.
Örneğin:
yüksekliği azaltmaya devam ederse:
hedefin
yalnızca bir
kısmı görünür
hale gelebilir.
Bu durumda görüntü işleme algoritması hedefi kaybedebilir.
Bu nedenle:
Hassas merkezleme ve alçalma sırasında hedefin görüntü içerisindeki görünürlüğü sürekli kontrol edilmelidir.
1m ve altında Hedefin o anda görüntüden çıkması durumunda ise sistem hedef yok diyerek gorevi iptal etmemelidir.

14. Hedef Üzerinde Son Konum
İHA hedefin merkezini görüntü işleme ile bulduğunda:
	hedef merkezini kamera görüntüsünde belirlemeli
	kamera merkezi ile hedef merkezi arasındaki pixel offset’i hesaplamalı
	bu offset’i yatay hareket komutlarına çevirmeli
	hedef merkezine hassas şekilde hizalanmalıdır.
Son yaklaşım aşamasında:
Zemin ↔ İHA
mesafesi:
1 metre
olduğunda hassas iniş durdurulmalıdır.
Bu aşamada hedefin tamamının kamerada görünür kalması özellikle önemlidir.

15. Hedef GPS Konumunun Kaydedilmesi
İHA hedefin merkezine hizalandığında hedefin GPS koordinatları kaydedilmelidir.
Örneğin:
target.latitude
target.longitude
target.type
Bu bilgi Görev 2’de kullanılacaktır.

16. Görev 1 – Yük Bırakma
Hedef tipi belirlendikten sonra ilgili yük bırakılır.
Yük eşleşmesi:
HEXAGON hedef
    ↓
RED payload

TRIANGLE hedef
    ↓
BLUE payload
Yani:
Tespit edilen alan	Bırakılacak yük
Mavi altıgen	Kırmızı yük
Kırmızı üçgen	Mavi yük
Yük bırakma işlemi dummy servo fonksiyonu üzerinden gerçekleştirilmelidir.
İHA kesinlikle disarm edilmemelidir.

17. İkinci Hedefi Bulma
İlk hedefe yük bırakıldıktan sonra sistem ikinci hedefi aramaya geçer.
İlk hedef zaten kayıtlı olduğu için sistem:
İlk hedef = biliniyor
İkinci hedef = henüz bulunmadı
durumunu bilmektedir.KabaKaba
Dolayısıyla görüntü işleme sırasında ilk hedef tekrar görülürse bu hedef daha önce işlendiği için yok sayılabilir.
Amaç ikinci, henüz işlenmemiş hedefi bulmaktır.

18. İkinci Hedefin Tespiti
İkinci hedef de ilk hedef ile aynı algoritma kullanılarak bulunmalıdır:
Kamera
   ↓
Görüntü işleme
   ↓
Renk tespiti
   ↓
Şekil tespiti
   ↓
Hedef doğrulama
   ↓
Hedef kaydı
   ↓
GPS koordinatının kaydı
   ↓
Kaba yaklaşma
   ↓
Hassas merkezleme
   ↓
1 m irtifa
   ↓
İlgili yükün bırakılması
İkinci hedefe de uygun yük bırakıldığında Görev 1 tamamlanmış olur.

19. Görev 2 – İlk Alana Geri Dönüş
Görev 1 tamamlandığında sistem artık ilk hedefin GPS koordinatına sahiptir.
Bu nedenle ilk hedefi tekrar aramak için tüm alan taranmamalıdır.
Doğrudan kayıtlı GPS konumuna gidilmelidir.
Kayıtlı GPS
    ↓
Hedef bölgesine git
    ↓
Görüntü işleme
    ↓
Hedefi tekrar bul

20. Yükün Bulunması
İlk hedef alanının rengi bilindiği için hangi yükün aranması gerektiği de bilinir.
Yük rengi hedef alanının tersidir.
Örneğin:
Mavi alan → Kırmızı yük

Kırmızı alan → Mavi yük
Yükler küçük kare objeler şeklinde modellenmiştir.
Yaklaşık boyutları:
10 cm × 10 cm
civarındadır.

21. Yük Tespiti – Kritik Problem
İHA 1 metre seviyesine yaklaşırken hedef alanın kendisi kameranın görüşünden çıkabilir.
Bu durumda görüntü işleme algoritması:
"Alan bulunamadı"
sonucuna düşmemelidir.
Çünkü bu aşamada sistem artık alanı değil, alanın üzerinde bulunan yükü aramaktadır.
İHA düşük irtifada hedef alanı göremese bile yük arama durumunda olduğunu bilmelidir.

22. Yükün Hassas Merkezlenmesi
Yük tespit edildiğinde İHA yükün merkezini görüntü içerisinde belirlemelidir.
İHA:
	yatay olarak yük merkezine hizalanmalı
	hassas iniş gerçekleştirmeli
	yükün tam üzerine gelmelidir yoksa yuk yerden alınamaz.
İniş sırasında:
İHA hiçbir koşulda DISARM edilmemelidir.
İHA yalnızca fiziksel olarak zemine/yüke temas etmelidir inis yapabiilir yapmalıdır fakat disarm olmamalıdır.

23. Yük Alma
İHA doğru konumda ve zemine temas etmiş durumda olduğunda pickup mekanizması çalıştırılmalıdır.
Gerçek servo/PWM mekanizması Gazebo’da modellenmediği için:
pickup_payload()
gibi bir abstraction kullanılmalıdır.
Bu fonksiyon gelecekte gerçek servo kontrolü ile değiştirilebilir.

24. Yükün Alınıp Alınmadığının Doğrulanması
Gerçek sistemde yükün alınıp alınmadığını doğrudan bildiren özel bir sensör bulunmamaktadır.
Bu nedenle doğrulama kamera üzerinden yapılacaktır.
Yük alma işleminden sonra:
Pickup
   ↓
İHA 1 metre yükselir
   ↓
Kamera tekrar aşağı bakar
   ↓
Yük görüntü işleme ile kontrol edilir

27. Yük Alma Başarısızsa
Yükün alınamadığı tespit edilirse Görev 2 içerisindeki pickup süreci yeniden denenmelidir.
Deneme sayısı kodun başında configurable variable olarak tanımlanmalıdır:
MAX_PICKUP_ATTEMPTS = 5
Akış:
Yük tespit et
      ↓
Merkezle
      ↓
İn
      ↓
Pickup mekanizmasını çalıştır
      ↓
1 m yüksel
      ↓
Kamera ile doğrula
      ↓
Başarılı?
   /       \
 EVET      HAYIR
  ↓          ↓
Görev 2    attempt++
tamamla      ↓
          5 deneme?
          /      \
       HAYIR     EVET
        ↓          ↓
     tekrar      Görev 2
     dene       başarısız
                   ↓
                  RTL
5 denemeden sonra yük alınamazsa Görev 2 başarısız kabul edilir.
Bu durumda sistem yine DISARM yapmamalıdır.
Doğrudan Görev 3’e geçilmelidir.

28. Görev 3 – Return To Launch
Görev 2 başarıyla tamamlandığında veya pickup işlemi 5 denemede başarısız olduğunda sistem Görev 3’e geçmelidir.
Görev 3:
Return To Launch (RTL)
olarak uygulanacaktır.
RTL sırasında İHA başlangıç/kalkış konumuna geri dönmelidir.
Görev 3 tamamlanmadan otomatik disarm yapılmamalıdır.


31. Görüntü İşleme Gereksinimleri
Görüntü işleme sistemi en azından şu işlemleri gerçekleştirmelidir:
Hedef alanları
1.	RGB/BGR görüntü al
2.	Uygun renk uzayına dönüştür (ör. HSV)
3.	Mavi/kırmızı renk maskeleri oluştur
4.	Gürültü azaltma
5.	Contour tespiti
6.	Contour geometrik özelliklerini hesapla
7.	Shape approximation uygula
8.	Üçgen/altıgen sınıflandırması yap
9.	Renk + şekil doğrulaması yap
10.	Bounding box/centroid hesapla
11.	Detection confidence oluştur
Yük
Benzer pipeline kullanılabilir fakat yükün geometrisi ve boyutu hedeflerden farklı olduğundan ayrı bir detector kullanılmalıdır.

33. Uçuş Optimizasyonu
İHA’nın batarya kapasitesi sınırlı olduğu için görev sadece doğru sonuç üretmeye değil, aynı zamanda mümkün olduğunca kısa sürede tamamlanmaya çalışmalıdır.
Özellikle:
	gereksiz hover
	gereksiz dönüş
	gereksiz tekrar taraması
	yüksek irtifada uzun süre hassas merkezleme
	hedef bulunduğu halde gereksiz alan tarama
engellenmelidir.
Ancak hız uğruna hedef tespit güvenilirliği feda edilmemelidir.
Öncelik:
Güvenli görev tamamlanması
        >
Gereksiz uçuşun azaltılması
        >
Maksimum hız
hız ve ivme parametreleri de kolayca degistirilebilir olmalıdır

34. Hedef Arama Algoritması
30 × 100 m alan için sistematik coverage path planning uygulanmalıdır.
Başlangıçta uygun aday olarak:
Lawnmower / Boustrophedon Coverage Path Planning
değerlendirilebilir.
Tarama hattı aralığı:
SCAN_LINE_SPACING
şeklinde configurable olmalıdır.
Bu değer doğrudan rastgele seçilmemeli; kamera:
	yatay FOV
	dikey FOV
	çözünürlük
	uçuş irtifası
	hedef minimum boyutu
	gerekli overlap
parametrelerinden türetilmelidir.
Tarama rotası hedefi kaçırmayacak kadar sık, ancak gereksiz tekrar oluşturmayacak kadar seyrek olmalıdır.

35. Sistem Mimarisi
Kod mümkün olduğunca aşağıdaki gibi modüllere ayrılmalıdır:
Bu yapı zorunlu olmakla birlikte dosya isimleri değiştirilebilir; temel amaç sorumlulukların ayrılmasıdır.
config dosyaları vb vb...

36. Veri Yapıları
Hedefler için merkezi bir veri yapısı oluşturulmalıdır.
Örneğin:
@dataclass
class Target:
    target_id: int
    shape: str
    color: str
    latitude: float
    longitude: float
    confidence: float
    detected: bool
    payload_released: bool
Böylece sistem:
Target 1
    shape = HEXAGON
    color = BLUE
    payload = RED
    payload_released = True

Target 2
    shape = TRIANGLE
    color = RED
    payload = BLUE
    payload_released = True
gibi bir durumu merkezi şekilde takip edebilir.

37. Görev Başarı Kriterleri
Görev 1 başarılı
Aşağıdaki şartların tamamı gerçekleşmelidir:
	İHA kalkış yaptı.
	Birinci hedef tespit edildi.
	Birinci hedefin şekli doğru sınıflandırıldı.
	Birinci hedef GPS konumu kaydedildi.
	Birinci hedefin merkezine doğru bir sekilde yük bırakıldı.
	İkinci hedef tespit edildi.
	İkinci hedefin şekli doğru sınıflandırıldı.
	İkinci hedef GPS konumu kaydedildi.
	İkinci hedefein merkezine doğru bir sekilde yük bırakıldı.

38. Görev 2 başarılı
	İlk hedefin GPS konumuna geri gidildi.
	İlk hedef tekrar doğrulandı.
	Hedefin üzerindeki doğru yük tespit edildi.
	İHA yükün üzerine tam merkezine hassas şekilde indi.
	Pickup mekanizması çalıştırıldı.
	İHA tekrar yükseldi.
	Kamera ile yükün alındığı doğrulandı.

39. Görev 2 Başarısızlığı
Pickup işlemi:
MAX_PICKUP_ATTEMPTS = 5
denemede de başarısız olursa:
TASK_2_FAILED
    ↓
RTL
uygulanmalıdır.
İHA disarm edilmemelidir.

40. Görev 3
Görev 3’ün tek amacı:
Return To Launch
gerçekleştirmektir.
Görev 3 tamamlandığında görev sonlandırılabilir.
Disarm davranışı, kullanılan ArduPilot/SITL görev mantığına uygun şekilde yalnızca görev sonlandırma aşamasında ele alınmalıdır.

41. Önemli Tasarım İlkeleri
Sistemin aşağıdaki hatalara karşı dayanıklı olması gerekmektedir:
Hata 1 – Hedef geçici olarak kayboldu
İHA hemen rastgele başka bir duruma geçmemelidir.
Kısa süreli görüntü kaybında:
TARGET_TRACKING
mantığı kullanılmalıdır.

Hata 2 – Hedef kameranın dışına çıktı
İHA kör şekilde alçalmaya devam etmemelidir.
Güvenli bir irtifaya geri çıkıp yeniden hedef edinme yapılmalıdır.

Hata 3 – Yanlış hedef tespiti
Tek bir frame üzerinden karar verilmemelidir.
Birden fazla ardışık frame üzerinde:
	renk
	şekil
	alan
	boyut
	centroid
tutarlılığı kontrol edilmelidir.

Hata 4 – Aynı hedef tekrar görüldü
Daha önce işlenmiş hedef tekrar bulunduğunda ikinci hedef olarak kabul edilmemelidir.
Hedef kayıt sistemi kullanılmalıdır.

Hata 5 – Yük yerde mi, İHA’da mı?
Sadece renk tespitine güvenilmemelidir.
Pixel boyutu + bounding box + yük geometrisi + irtifa birlikte değerlendirilmelidir.

Hata 6 – Pickup başarısız
Deneme sayısı:
MAX_PICKUP_ATTEMPTS = 5
ile sınırlandırılmalıdır.
5 başarısız deneme sonrasında RTL uygulanmalıdır.

Hata 7
gorevler arasında gecis yaparken ihanın yaptıgı pitch ve roll hareketleri kameranın direkt olarak zemini degilde baska yerleride gormesine sebep olabilir. roll ve pitch yaptıgında hedef tespit ederse kafası karısmamalıdır onu tam altında sanmamalıdır

42. Özet Görev Akışı
Tam görev akışı aşağıdaki gibi olmalıdır:
START
  │
  ▼
TAKEOFF 10m
  │
  ▼
İlk hedefi ara
  │
  ├── Bulundu ───────────────┐
  │                           ▼
  │                    Şekil + renk belirle
  │                           │
  │                           ▼
  │                    Hedefi kaydet
  │                           │
  │                           ▼
  │                    Hedefe yaklaş
  │                           │
  │                           ▼
  │                    Hassas merkezle
  │                           │
  │                           ▼
  │                      1m'de dur
  │                           │
  │                           ▼
  │                    GPS kaydet
  │                           │
  │                           ▼
  │                    Uygun yük bırak
  │                           │
  │                           ▼
  └── Bulunamadı → Alan tarama
                              │
                              ▼
                       İkinci hedefi ara
                              │
                              ▼
                       Şekil + renk belirle
                              │
                              ▼
                         Hedefi kaydet
                              │
                              ▼
                         Hedefe yaklaş
                              │
                              ▼
                        Hassas merkezle
                              │
                              ▼
                             1m
                              │
                              ▼
                       Uygun yük bırak
                              │
                              ▼
                    ┌────────────────────┐
                    │     GÖREV 1 BİTTİ  │
                    └────────────────────┘
                              │
                              ▼
                  İlk hedef GPS'e git
                              │
                              ▼
                     İlk alanı doğrula
                              │
                              ▼
                      Ters renkli yükü ara
                              │
                              ▼
                       Yükü tespit et
                              │
                              ▼
                        Yükü merkezle
                              │
                              ▼
                         Hassas in
                              │
                              ▼
                      Pickup mekanizması
                              │
                              ▼
                           1m yüksel
                              │
                              ▼
                       Yükü doğrula
                         /         \
                      BAŞARILI     BAŞARISIZ
                        │             │
                        ▼             ▼
                GÖREV 2 BİTTİ    attempt++
                                      │
                                      ▼
                              attempt < 5 ?
                                /       \
                              EVET      HAYIR
                               │          │
                               ▼          ▼
                         Tekrar dene     RTL
                                          │
                                          ▼
                                         RTL

43. Sonuç
Bu sistemin temel amacı yalnızca hedefleri görüntü işleme ile bulmak değildir.
Sistem:
1.	Bilinmeyen başlangıç konumundan kalkmalı,
2.	30 × 100 m alan içerisinde hedefleri sistematik şekilde aramalı,
3.	Mavi altıgen ve kırmızı üçgeni ayırt etmeli,
4.	Hedeflerin GPS konumlarını kaydetmeli,
5.	Hedeflere doğru yükleri bırakmalı,
6.	İlk hedefin kayıtlı GPS konumuna geri dönmeli,
7.	Hedef üzerindeki küçük yükü ters renk bilgisi kullanarak tespit etmeli,
8.	Yükün üzerine hassas iniş yapmalı,
9.	Dummy pickup mekanizmasını çalıştırmalı,
10.	Görüntü işleme ile yükün alınıp alınmadığını doğrulamalı,
11.	Gerekirse pickup işlemini maksimum 5 kez tekrarlamalı,
12.	Başarılı veya başarısız olmasından bağımsız olarak Görev 3 kapsamında RTL gerçekleştirmeli,
13.	Görev 3 tamamlanana kadar hiçbir koşulda disarm edilmemelidir.
Kod mimarisi, modüler, okunabilir, parametreleri merkezi olarak tanımlanmış ve ileride Gazebo'daki dummy servo/pickup fonksiyonlarının gerçek PWM/servo kontrolüyle değiştirilebileceği şekilde tasarlanmalıdır.
log tutmalı ve terminale suan hangi asamada ve ne yapmakta oldugunu anlık olarak göstermelidir.
