

# Zagadnienia i założenia projektowe

Postawione przed nami zadanie polegało na zaprojektowaniu regulatora PID i DMC, które sterują obiektem zrealizowanym na mikrokontrolerach z serii STM32. Powinniśmy tak manipulować sygnałem wejściowym procesu *u*, aby wartość sygnału wyjściowego procesu (regulowanego) *y* była możliwie bliska wartości zadanej *y^{zad}*. Wartość uchybu *e=y^{zad} - y* powinna być jak najmniejsza. Wyniki uzyskane podczas eksperymentów zostaną porównane i poddane krytycznej weryfikacji. 

## Omówienie implementacji

Tradycyjnie regulację za pomocą algorytmu PID realizujemy za pomocą trzech członów proporcjonalnego, całkującego i różniczkującego. Człon proporcjonalny powoduje wzrost wartości sterowania wraz z wzrostem uchybu, całkujący zwiększa wartość sygnału sterującego wraz z akumulowanym uchybem, a dla różniczkującego wraz z wzrostem uchybu, wzrasta wartość sygnału sterującego. 

Implementacji algorytmu dokonaliśmy w plikach *Pid.c* i *Pid.h*. Parametry PID-a zostały w kodzie zaprezentowane jako struktura *Pid*. W pliku *main.c* zadajemy wartości odpowiednim parametrom z struktury PID. Wymagane obliczenia w algorytmie są realizowane za pomocą funkcji float pidCe(Pid *pid, float pv), której argumentami są struktura z wartościami naszego PID-a i zmienna *pv - process value*, czyli naszą wartość zadaną, a funkcja zwraca nam sygnał sterujący. 

W każdym wywołoniu funkcji dokonujemy następujących obliczeń: 

1. Wyliczamy uchyb na podstawie wzoru: e(k) = y^{zad}(k) - y(k)

2. Wartość członu proporcjonalnego  
   $$
   u(P)=Ke(k)
   $$

3. Wartość członu całkującego 
   $$
   u_{I}(k) = u_{I}(k-1) + \dfrac{K}{T_{I}}T\dfrac{e(k-1)+e(k)}{2}
   $$

4. _Wartość członu różniczkującego _
   $$
   u_{D}(k) = KT_{D}\dfrac{e(k)-e(k-1)}{T}
   $$

5. Następuje zapisanie wartości z stanu *k* jako wartości dla stanu *k-1* (w naszym kodzie zmienne z poprzedniego stanu wyrażone są za pomocą przedrostka prev)

Oprócz tych kroków do naszego algorytmu zastosowaliśmy rozwiązanie anti-windup. Rozwiązania tego używamy w przypadku gdy zmienna sterowania osiąga wartość graniczną urządzenia wykonawczego. Wiemy, że nie ma sensu zadawać większej wartości sygnału sterowania niż element wykonawczy jest w stanie zrealizować. W takiej sytuacji przerywamy pętlę sprzężenia zwrotnego i system zaczyna pracę w pętli otwartej. Takie rozwiązanie zapobiega ,,nawijaniu'' członu całkującego, czyli osiąganiu nadzwyczaj dużych wartości członu całkującego co prowadzi do ogromnego spowolnienia działania regulatora, a w skrajnych przypadkach do jego rozregulowania. 

Odp skokowa !!!!!!!!!!!!!!!!??????????????????????
Dostrajanie regulatora odbywało się na zasadzie pozyskiwania odpowiedz skokowej. Obserwując w jaki sposób sygnał sterujący generowany przez regulator osiąga wartość zadaną podczas skoku dokonywaliśmy oceny regulacji. W ten sposoób wybieraliśmy najlepsze nastawy dla regulatora. 

## Wyznaczanie nastawów regulatora metodą Zieglera-Nicholsa

Przbieg strojenia regulatora przy użyciu metody Zieglera-Nicholsa

1. Implementujemy regulator typu P.

2. Wartość wzmocnienia K dobieramy tak aby wyjście obiektu regulacji miało charakter oscylacyjny (nierosnący, niemalejący). Przyjmujemy wzmocnienie krytyczne 
   $$
   K_{u} = K
   $$
   Odczytujemy jeszcze okres oscylacji: 
   $$
   T_{u}
   $$
   ![zieglerkp](Zdjecia\zieglerkp.svg)

   Po wykonaniu tych punktów uzyskujemy powyższy wykres na którym wyraźnie widać oscylacje niegasnące i niemalejące wyjścia regulatora. Okres oscylacji to czas pomiędzy dwoma sąsiadującymi wierzchołkami lub dołkami. 

3. Używając tabelki wyliczamy parametry K, T__{I}, T_{D}  w zależności od regulatora który chcemy stosować P, PI, PID. My oczywiście wybiermay wzory dla PID.

   ​								 ![TabelaPID](Zdjecia\TabelaPID.PNG)

4. Wyznaczone parametry powinny zapewnić niezłą jakość regulacji, gdy będziemy chcieli spróbować znaleźć lepszy regulator zaczęcie od nastawów wyznaczonych metodą Zieglera-Nicholsa będzie dobrym pomysłem.

   ![ziegler_gotowy](Zdjecia\ziegler_gotowy.svg)

   Wykres przedstawia przebiegi sygnałów dla nastawów wyliczonych według tabelki. Jeżeli nie sterujemy mocno skomplikowanym obiektem i amplituda zmian wartości zadanej nie jest zbyt duża. To jakość regulacji możemy uznać za satysfakcjonującą. Wyjście obiektu bardzo szybko dochodzi do wartości zadanej i jest stabilne (nie oscyluje). Zmiana wartości sygnału sterującego na początku jest akceptowalna, a w późniejszym czasie (od próbki 60) zmiany mają charakter skoków o nie dużych amplitudach. Jest to wynik zakłóceń i szumów występujących w układzie. 

# Algorytm DMC

# Porównanie najlepszych realizacji PID i DMC

# Wnioski





















































\end{document}

