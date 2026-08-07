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

**oneri15**:Hayır surekli hata yapmaya devam ediyorsun yaptıgın tum degisikleri geri aldım. yine en bastan basladık. tekrar algoritmanın ve yazılım yapması gereken seyi seyi ve hangi sorunu cozdugunu anlatıyorum.
payload search adımı geldigi zaman eyer kamera aynı anda iki adet aranan renkten obje gorurse ikisininde payload oldugunu dusunuyor ve ne yapacagı konusunda kararsız kalıyor. ornegin hedeflerin cok yakın oldugu bir senaryoda payload ile diğer hedef in rengi aynı olacagı olacagı için hedefi payload sanabilir. bu durumu engellemek icin su sekilde bir algoritma gelistirilmeli.
payload search basladıktan sonra eger gorntu isleme birden fazla payload tespit ederse bu tespit etmis oldugu payloadlardan hangisi aktif olarak daha az pixel sayısına sahipse yani daha kucukse ona yonelmeli. ardından bu dogru bir sekilde kucuk olan yani gercek olan payloada dogru yonelirken irtifası 1 metreye ulastıgında bu karar vermis oldugu payload ın ekranda ne kadarlık bir alan kapladıgını hesaplamalı ve kaydetmeli. diyer adımda yani payloadı alıp almadıgını kontrol ederkende 1 metre irtifaya cıktıgı icin yine tespit ettigi bu payload ın alanını zaten biliyor olacak. eger yine bir payload oldugunu dusundugu bir sey tespit eder fakat bu tespit ettigi seyin alanı eklenen hata paylı haleden bile daha az veya daha buyukse bunun payload olmadıgı anlısılır ve payload ı basarıılı bir sekilde almıs oldugunu anlar.

**oneri17**: gornut ortalandıkan sonra 2m irtfada direkt yuku bırakmasına gerek yok goruntu islmeyi kapatıp irtifasını 1 metreye dusuru ortalanmıs kkonumdan daha sonra yuku bırakır. bu sekilde daha saglıklı



**oneri18**: 1m metre yukselip payload ı alıp almadıgnı kontrol ettigi kısımda hata yapıyor payload yerde olmasına ragmen kendisnin payload ı almıs oldugunu sanıyor kodun bu kısmında bir hata var. 