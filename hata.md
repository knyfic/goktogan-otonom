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