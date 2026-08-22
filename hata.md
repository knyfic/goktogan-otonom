**oneri1**: kamera nin cozunurlugu ve fov u degistiketn sonra eklenmis olan pixel boyut filtrelerinin calısması gozden gecirilmelidir bu filtreler gercek kamerada farklı sonuclar verebilir
---
**oneri2**: bir sekilde tespit edilir ve o sekil kadrajdan cıkarsa en son tespit edilen yerin gps kordinatına gidip bir alana taraması yapıyor sekli en son gordugu yerde. bu kısımda bulunan search algoritması daha iyi bir hale getirlmeli o sekli gordugu gps noktasının etrafında belirli bir yarı cap olarak cember cizmeli
---
**oneri3**: rtl altitude degistirilebilir.
---
oneri4: yerdeki kucuk bir copu surekli olarak mavi altıgen veya mavi ucgen gibi algılamasının onune gecmek icin eger tespit edilen sekil minimum kameradaki alan kriterini karsılamıyorsa yok sayılmalı.
---
**oneri5**: gorev eyer bir yerde stuck olur veya isleyemez hale gelirse
        genel olarak tum gorev icin  gecerli bir timer eklenip o sure icersinde tamamlanamazsa rtl olmalıdır
        genel olarak gorevlere de timer eklenebilir. ornegin x gorevi 10  atempt veya 40 saniyede tamamlanamazsa gorev iptal edilip rtl edilebilir
---
**oneri6**: payload retry altitude 1 metreye yukseltilmeidr
---
**oneri7**: yuk searching payload durumunda eger kadrajdan cıkarsa  iha searching payload durumda stuck olarak kalıyor oldugu yerde bu bir acık. bunun cozumu icin bir algoritma gelsitirlmeli
orniegn iha bu durumda tekrar 2 metre havaya kalkıp tekrar hedefi ortalamalı ve payload search a gecmei yani gorevi bastan almalı gibi. belirli bri timeout sonrası tabiki
---
**oneri8**: oneri7 suan calısıyor fakat yine eklenebilir eyer 20 saniye sonra yukarı cıktıgında da hedefi bulamazsa tekrar bir 20 sabiye o bounca arayacak. bu onun son arayısı olmalı. yani assagıda bekle 20 saniye beklemesin farklı bir config e veri atayıp dah kısa sure beklesin mesela 5 saniye. arıdan yukselip 20 saniye yukseldikten sonra arasın. eyer bulamazsa gorevi iptal etsin ve eve donsun

**oneri9**: payload i alırken boyut hesaplaması yapmalı. iki hedef birbirlerine cok yakınsa hedef1 de keki payloadta payload ı arkerken ki payload ın rengi hedef 2 ile aynı olacak yani hedef 2 yi eger hedef1 ortaladıktan sona goruyorsa search payload kısmındada gorecek demektir. bu tur bir durumda yapacagı sey su olmalı: sandıgı bu iki payloadtan hangisi daha kucukse ona yonelmeli. yoneldikten sonra payloadı almak icin assagıya dogru hareket yaparken irtfiası 1 metreye ulastıgnda arka planda payload ın 1 metre irtifada iken kapladııgı alan pixel kaydedilmeli. bu sayede eyer yuku aldıkta sonra validate yaparken 2.hedefi gorur ve ona yonelirse zaten 1 metrede onun ne kadar yer kapladıgı bildigi icn hedef ondan daha buyuk oldugundan bunun hedef olmadıgını anlayabilecektir. cozum yontemi bu sekilde. tabiki exact pixel sayıs olmak zorunda degil her nekadar kaydeilmis olsada pixel sayısı ufak tefek degiseblir false sinyal vereek vaidate de almadıgı halde aldım da sanmamalı bunu ayarlaman gerekiyor
payloadı alrıken eyer aynı anda iki hedef tespit ederse boyut hesaplaması yapmalı ve bu yaptıgı hesap sonucunda en kucuk tespit edilen payload ı gercek payload olarak isaretlemek ve ona dogru yonelmeli. yoneldikten sonra alcalma yaptıgı sırada eyer iltifas 1m ye ulasırsa o terich edilmis olan payloadın 1m de sahip oldugu alan hesaplanır. bu sayede quadcopter tekrar 1 metreye kalkıp kontrol ederekn eyer kontrol ettignde ki  eyer alan aynı degilse bunun yuk degilde tespit edilen diger yuk oldugu anlasılacak

**oneri10**: rtl complate olduktan sonra "2026-08-06 15:27:21,318 | INFO     | iha_mission | STATE: RTL → COMPLETE
fish: Job 1, 'python3 main.py' terminated by signal SIGSEGV (Address boundary error)" hatası aldım

**oneri11**: prearm good olana kadar beklemeli. kod prearm good olmadan once ccalısıtıralarsa stuck oluyor ve bozuluyor

**oneri12**: preamr good olduktan sonra kod calıstırasa bile takeoff olması cok uzun suruyor

**oneri13**: gorev bitince diretk kendi kendine kapanıyor

**oneri14**: 1 metre irtifada payloadların kapladıgı pixell alanı hesaplanabilir. bu sayede  aktif olarak bir ayırma algoritması gerekmeden direkt olarak ezbere bildigi alana sahip sekli arar ve bu alandan daha fazla veya az alana sahip yanlıs tespitleri kale almaz.

**oneri17**: gornut ortalandıkan sonra 2m irtfada direkt yuku bırakmasına gerek yok goruntu islmeyi kapatıp irtifasını 1 metreye dusuru ortalanmıs kkonumdan daha sonra yuku bırakır. bu sekilde daha saglıklı

**oneri19**: payload ı validaet etmesi icinde bir timeout olabilir diretk bulamazsa evet aldım dememeemli belki goruntu isleme anlık olarak bir aksaklıga ugramıs olabilir

**oneri20**: bir kod daha olmalı ve bu kodun tek amacı ana gorevde bir sekilde cokme yasanırsa rtl yi tetiklemek olmalı oldugu yerde bosuna ugrasmasın veya kodu tekrar bir sekilde basltabilir. gorevin kaldıgı yer ve sekil payload gibi bilgeleri formatta saklanarak tekrar ana pyhton dosyası baslatıglıdıgn kullanabilir. + olarak ardupilot un parametreleri hakkında da arastırma yapılmalı belki rasberry pi dan gelen heart beat in kesilmesi sonucu rtl ye falan girebilir bunun olmasını istemiyoruz.

**oneri19**: yuklerin bırakılması ksımında iha burnunu illa seklile ortalamk zorunda kalmamalı bu onun havada bos yere yaw yapmasına sebebp oluyor o anki oryantasyonu ne ise o sekilde isledigi gorntuye yaklassınve yukleri bıraksın yukle zaten ihanın ortasında bulunuyor

**oneri20**: yalnızca gercek payloadın boyut hesabı yapılmadı fakat suanki algoritma ekranda aynı renkte gordugu her seyin boyut hesabını yapıyor

**oneri21**: payloadı almak icin ilk hedefin konumuna dogru gidildigi sırada irtifanın 2 metre olmalı cunku goruntu isleme 1 metreden seklin tamamenı goremiyoru 2 metre olması daha iyi. 

**öneri22mekanik**: yukler yarısma anında yerlestirelcegi icin hangi renk yukun hangi mıknatısa yerlestirilecegi karıstırılmaması amacıyla yuk mekanizmasına isaretler konulmalıdır.

**oneri23**: iha payloadı almak icin payloadı ortalarken 0.4m de ortalıktan sonra direkt assagıya inis yapıyor. bu olmamalı. yere yakın ken tekrar ortlaması gerekiyor 0.2metrede iken falanda ortlaması lazım. 0.4 te ki ortalaması basarılı olmuyor

**oneri24**: kod rtl ye girdigi zaman nedense bir lag giriyor goruntu isleme penceresi kasmaya baslıyor bunun sebebi bulunmalı

**oneri25**:
```
STATE: RELEASING_RETURNED_PAYLOAD → RTL
2026-08-08 16:57:57,440 | INFO     | iha_mission | RTL tamamlandı.
2026-08-08 16:57:57,440 | INFO     | iha_mission | STATE: RTL → COMPLETE
Fatal Python error: _PyThreadState_DeleteCurrent: the function must be called with the GIL held, after Python initialization and before Python finalization, but the GIL is released (the current Python thread state is NULL)
Python runtime state: finalizing (tstate=0x0000000000ba6748)

Thread 0x00007061c0a59080 (most recent call first):
  <no Python frame>

Extension modules: numpy.core._multiarray_umath, numpy.core._multiarray_tests, numpy.linalg._umath_linalg, numpy.fft._pocketfft_internal, numpy.random._common, numpy.random.bit_generator, numpy.random._bounded_integers, numpy.random._mt19937, numpy.random.mtrand, numpy.random._philox, numpy.random._pcg64, numpy.random._sfc64, numpy.random._generator, cv2, google.protobuf.pyext._message (total: 15)
fish: Job 1, 'python3 main.py' terminated by signal SIGABRT (Abort)
```

**oneri26**: payloadı almaya giderken tekrar yukselmeli ve ardında goruntu isleme ile hedefi ortalamalı mevcut irtfida dusuk oluyor. tıpkı returning payload da oldugu gibi.

**oneri27**: zamanlaycı python kodu baslar baslamaz baslamalı. takeoff falan beklemememli program acıldıgı andan itibaren zaman calısmalı.

**oneri28**:rasberry pi ile connection refused duruma dustugu gibi program cokuyor iha rtl falan yapmalı belirli bir timeout boyunca sinyal gelmez ise vb vb... 

**oneri29**: kameranın sallanması saglayacak olan gerekli fiziksel parametrelerini ayarlanabilir. bu sayede sim daha tuturlı bir sekilde gercegi yansıtır.

**oneri30**: step motorun adımlarını tamamlaması cok uzun suruyor "2026-08-09 15:51:32,004 | INFO     | payload_service | Stepper ↓ aşağı: 17143 adım
2026-08-09 15:51:32,004 | INFO     | payload_service | Yanıt gönderildi → 192.168.1.5:57694
2026-08-09 15:51:35,070 | INFO     | payload_service | Bağlantı kabul edildi: 192.168.1.5:57698
2026-08-09 15:51:35,071 | INFO     | payload_service | Komut alındı: FINISH_PICKUP  params={}
2026-08-09 15:52:05,071 | INFO     | payload_service | Pickup tamamlandı, Stepper↑
2026-08-09 15:52:05,071 | INFO     | payload_service | Stepper ↑ yukarı: 17143 adım" (hil modunda rasberrypi da calısan kodun cıktısı)
yani ana kodun bu kısmında "2026-08-09 15:51:31,744 | INFO     | iha_mission | [REMOTE] send_pickup_command
2026-08-09 15:51:32,019 | INFO     | iha_mission | Pickup servo komutu gönderildi (RED). 3.0s bekleniyor...
2026-08-09 15:51:35,024 | INFO     | iha_mission | [REMOTE] finish_pickup
2026-08-09 15:52:41,432 | INFO     | iha_mission | PAYLOAD PICKUP TAMAMLANDI: RED" goruntu isleme ve sistem tam anlmaıyla donuyor servo job ı bitine kadar. bu gercekten guzel servoyu beklemesi zaten bizim ihtiyacımız olan bir sey fakat sorun şu: ana kod serovyu beklesin o job u dinlesin fakat donmasına gerek yok. + olarak artık fizksel bir servo ile test ettigimiz icin yapay bir 3 saniye beklniyor yazısına ihtyacımız yok hala simler icin 3 saniye beklesin fakat pencerede yazmasına gerekyok.


**oneri31**: ilk yuk mavi yuk yani kırmızı ucgen hedefe bırakılacak olan yuk oldugu zaman payload bırakma gorevi basarılı bir seklde tamamnalmıyor yuk bırakılmıyor. mıktanıslar aktif kalmaya devam ediyor. fakat kod bıraktıgını sanıyor.

**oneri32**: ornegin ilk kırmızı hedefe yoneldikten sonra araması gereken seyin mavi altıgen oldugunu kodunbilmesi gerekior payload bırakma gorevi sırasında. cunku iki adet alanımız var birisi kırmızı cugen diger mavi altıgen eyer ilk yuku zaten kırmızı ucgene bırakmıssı ikinci alanı mavi altıgen olmak zorunda. kafa karısıklıgı yasmamalı baska bir kırmızı ucgen goruncu ona yonelmemeli cunku aradıgı sey artık mavi altıgen. aynısı tam tersi icinde gecerli

**soneri33**: step sasyısı ve step yonu ayarlanmalı

gorev basladıktan hemen sonra  arm olmak yerine belirli bir sure gorev beklemelibu sayede mıktanısların yukleri tuttugundan emin olunur.

arducopterin parametrelerini değiştiren veya aşan kodlar bulunabilri bunlar tespit edilmeli.

fps yazılacak
kamera kaydı gerceye yakın alınacak

- konturun tamamı görüntüde mi? sorusu sorulabilri bu sayede kırmızı ucgen kırmızı kare ile karıstırlmaz
iha once objeye yonelebilir objenin tamamı goruntude olduktan sonra kare mi ucgen mi oldugunu anlar eger devam etmesi gereken line a geri donerek search gerceklestirir.

garip bir hata: ik basta araması gereken sekil kırmız sekidle fakat kırmızı sekill tespit edilir ve ardından kadrajdan cıkar ve tam bu esnada eger mavi sekilde kadrajda ise mavi sekil tespit edilir. halbuki en basından araması gereken sekilde zaten kırmızı bir ucgen oldugu icin neden kırmızı ucgen kadradaj nbir anlıgna cıkınca mavi altıgeni de targetliyor ki? baska bir durumda mavi altıgeni targetlamıyor.

8m irtifadan 10cm lik bir yuku gormesi imkansiz payload zaten kırmızı payload zaten kırmızı ucgenden alınacagı icin payloada gideceegi irtifa dusurulebilir payload search altitude degistirilerek duzeltilebilir.

ilk payloadı kırmızı ucgene bırakacgı icin payloadı bıracak oldugu irtifanın 8m olmasına gerek yok
iki ayrı payload bırakma irtifası(daha doguru son goruntu islemenin yapılacagı irtifa)
iki ayrı offset duzetlme degeri (birisi kırmızı digeri ise mavi hedef icin)
yani returning payload kısmında da mavi altıgen icin tanımlanmıs offset degerleri kullanılacak
