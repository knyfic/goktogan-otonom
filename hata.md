hata7: payloadı alırken tam irtası neredeyse 0 oldugu konumda hem drone un kendisnde hem de kamerada bir donma yaasınıyor sanki bir lag giritormus gibi bu duzetilmeli

hata8: payloadlar 10cmlik karelerden olusuyor yani cok kucukler iha sureki olarak kendisini ortalaya ortalaya kademeli bir sekilde alcalamalı tam olarak exact 10cm lik bri hassasiyetle yukun uzerine inis yapabilmeli. yavas olması sorun degil amac yuku alabilmek.

hata9: hedeflerin konumları yuk bırakılmadan hemen once kaydedilmeli. cunku daha erken safalarda kaydedilerse hedefin tam ortası kaydedilmis olmuyor tam olarak ortalandıktan emin olundunta sonra konum kaydeidlmi ki kaydedilen konum hedefin tam ortası olsun

hata9: searching payload adımına gecmeden once ilgili konuma gelteinte hemen once kısa bir sekil kontrolu yapmalı ve kendisini once hedefin ortasına ortalamalı daha sonra ortalındak sonra hedefin sutundeki yuku aramalı

hata10:iki hedefi aynı anda gordugunde yalnızca 1 ine kilitlenmesini saglaycak olan algoritma guzel calısıyor bu sekilde kilitlenmeler onlenebildi. fakat suanda sıkıntı su o 1 saniyelik karar verme asması bitmedenn once iha yinede iki hedefede aynı anda bir saga bir sola gidecek sekilde yalpalanıyor. karak verilmeden once harekete gecilmemeli.

hata11: payload in yine cok kucuk olamsından oturu iritifa 2metrede civarllarında bile olsa payload ı tam olarak ortlayamadıgı icin sistem stuck kalıyor su sekilde "     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.97m hizalı=False
2026-08-05 01:31:01,793 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:02,218 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:02,621 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:03,043 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:03,467 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:03,868 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:04,269 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:04,670 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:05,095 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:05,520 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,231) dx=17 dy=9 irtifa=1.98m hizalı=False
2026-08-05 01:31:05,943 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:06,365 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.97m hizalı=False
2026-08-05 01:31:06,786 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:07,208 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:07,629 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:08,050 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False
2026-08-05 01:31:08,472 | INFO     | iha_mission | PAYLOAD HİZALA: merkez=(337,230) dx=17 dy=10 irtifa=1.98m hizalı=False" bu yuzden bu sorun cozulmeli. payload algoritması da suan guzel durumda sadece bu sorun cozulmesi gerekiyor. mesafe 1 metre ve altına dusmedigi surece bu denli hassas olmasına gerek yok gibi.


tespit edilen seklin merkeze yakınlıgıda hesaba katılabilir hangisinin once tercih edilecegine karar verilirken

kameranın merkezi ile goruntusu islenen cimsin merkezinde ki mesafe cok az ise drone tam anlamıyla nedense stuck oluyor. oylece bekliyor o yerde. asıl yapması gereken sey yavas haraketlerle fark ne kadar kucuk olursa olsun olabildigince sıfıra indimrye calısmak olmalı payloadı alırken. stuck olmak ve sihirli bir sekilde cismin kendi kendi merkeze lamasını beklemek degil.










hata1: ilk yuku bıraktıktan sonra direkt olarak search algoritmasına girmememesi gerekiyor once hedef irtiafa olan 8 metreye yukselmeyi beklmeesi ve daha sonra search yapması gerekiyor
hata2: gps konumnu kaybettigi hedef noktaya ilerledikten sonra goruntu islemeyi aktiflestirmesi gerekiyoro bu çalışıyor fakat goruntu islemeyi de o anki irtifadayken aktiflestirmesi gerekiy or dirtafa dayken aktiflestirmesi gerekiyor once 2m metre irtifasy inip degil bu yanlıs  ilgili gps konumu geldikten sonra sahip oldugu irtifa ne ise o irtifada direkt goruntu islemeyi baslatmalı once irtifasına azaltmamalı bu onun gorus acısını kısıtlıyor cunku
---

hata1:payload searching de bir hata var. ornegin iki hedefte birbirne cok yakında hedefler birbilerine zıt renkte oldugu icin payloadı araken yanlısıkla yuke degilde hedefe yonelebilir bunun olmasını istemiyoruz bunu onlemek icin sekil ortalandıktan ve payload search basladıktan sonra goruntu isleme ile tespit edilen seyin boyut hesabı yapılmallı. Eger buyuk ise sonuc olarakk bu bir payload degil hedeftir.(payloadlar 10cm kenara sahip karalerden olusuyor)

hata2: yuk bırakımı sırasında hedefe dogru kademeli alcalma yapılırken nedense alcama yapacgı sıra iha burnunu bir baska yone yoneltiryor ve alcama yapıyor alcama bitikte sonra tekrar burnunu seklin merkezie bakacak sekilde donderiyor. bos yere nedne ilk basta hedeften baska bir konuma donduruyor burnunu alcalma yaparken?

hata3: iha kamearaını izlemzi saglayan ekran guncellestirlmeli daha iyi bir hale getirlmeli bilgiler okunaklı olmaı + olarak ust ekrnaın ust kısmında var olan siyah renkli opak serit tam ekranı izleyemieze sebep oluyor biraz saydam olmalı
