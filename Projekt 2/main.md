% SMS - Projekt 2
% Krystian Chachuła
  Marcin Dolicher
% 15. grudnia 2018

# Zadanie

Postawione przed nami zadanie polegało na stworzeniu regulatora DMC, który miał utrzymywać temperaturę na stanowisku grzejąco-chłodzącym. Głównym celem tego projektu było jednak stworzenie interfejsu operatora z uwzględnieniem dobrych praktyk w projektowaniu HMI oraz reakcją na stany awaryjne.

\newpage

# Obiekt regulacji

Obiektem regulacji było stanowisko grzejąco-chłodzące. Obiekt ten jest opisany   w poniższej tabeli.

sygnał                          opis                                     min                         max
---------------                 --------------------------------------  --------------------------- -----------------------
wyjście obiektu                 temperatura mierzona sondą T1           $-55\;\text{\textdegree}C$  $+125\;\text{\textdegree}C$
wejście obiektu                 średnie napięcie na grzałce G1          $-50\;\%$                   $+50\;\%$
zakłócenia (wej. niesterowane)  prędkość kątowa łopatek wentylatora W1  n/d                         n/d

![Stanowisko termiczne, schemat[^1]](Zdjęcia do sprawka/term_sch.jpg){ width=75% }

![Stanowisko termiczne, zdjęcie[^1]](Zdjęcia do sprawka/term.jpg){ width=75% }

[^1]: Źródło: K. Czerwiński, S. Plamowski, A. Wojtulewicz; Laboratorium przedmiotu Systemy DCS i SCADA

# Odpowiedź skokowa

Regulator DMC używa modelu obiektu w postaci odpowiedzi skokowej. Jest to odpowiedź obiektu na skok sygnału sterującego równy +1. Aby ją uzyskać, w stanie ustalonym, zwiększyliśmy wartość sygnału sterującego obiektem z $-20$ na $30$ i zarejestrowaliśmy próbki wartości wyjścia obiektu wraz z wartością sygnału sterującego w celu rozstrzygnięcia chwili skoku w późniejszej analizie. Eksperyment ten powtórzyliśmy w celu weryfikacji otrzymanych wyników.

Następnie ze wszystkich próbek wybraliśmy te, które zostały zarejestrowane po skoku sygnały sterującego. Pierwsza rozpatrywana próbka była następną po skoku.

![Odpowiedź obiektu na skok sygnału sterującego z $-30\;\%$ na $20\;\%$](Zdjęcia do sprawka/rawresp.png){ width=75% }

Kolejnym krokiem była normalizacja odpowiedzi obiektu. Próbki wyjścia procesu pomniejszyliśmy o wartość pierwszej z rozpatrywanych. Aby zakończyć normalizację, podzieliliśmy rozpatrywane próbki przez $50$, ponieważ o tyle zwiększyliśmy wartość sygnału sterującego podczas eksperymentu.

![Odpowiedź skokowa (po normalizacji)](Zdjęcia do sprawka/stepresp.png){ width=75% }

Do dalszych rozważań wybraliśmy odpowiedź z eksperymentu pierwszego, ponieważ nie zauważyliśmy istotnych różnic między wynikami obydwu eksperymentów.

# Algorytm DMC

Z wykresu odczytaliśmy horyzont dynamiki obiektu tzn. liczbę kwantów czasu (próbek) potrzebnych obiektowi do ustabilizowania się po skoku sygnału sterującego. $y$ wydaje się przestawać rosnąć w okolicach $600$ próbki, zatem przyjęliśmy $D=600$. Horyzonty sterowania i predykcji przyjęliśmy równe horyzontowi dynamiki, a wartość kary za zmiany sygnału sterującego równą jeden, ponieważ tak dobrane parametry zazwyczaj pozwalają uzyskać dobrą jakość regulacji.

$$
D = 600
$$

$$
N_u = N = D
$$

Naszym kolejnym krokiem było wyznaczenie współczynników prawa regulacji za pomocą skryptu udostępnionego przez dr. Piotra Marusaka.

Implementacja samego regulatora została przeniesiona z poprzedniego projektu z jedną modyfikacją. Tym razem zastosowaliśmy *informowanie* regulatora o ograniczeniu wartości sygnału sterującego. *Informowanie* to polega na dopisywaniu do wektora przeszłych przyrostów sygnału sterującego delty takiej, która nie pozwoli temu sygnałowi przekroczyć ograniczeń, a nie delty wynikającej wprost z prawa regulacji. Zapobiega to nadmiernej akumulacji wartości sygnału sterującego w sytuacjach anormalnych. Zostało to zrealizowane w następujący sposób
```c
// Wylicz deltauk z prawa regulacji i przesuń bufor dmc->deltaup w prawo
// ...

// Ograniczenia
dmc->deltaup[0] = deltauk;
float uk = fmin(fmax(dmc->uk + dmc->deltaup[0], dmc->min_u), dmc->max_u);
dmc->deltaup[0] = uk - dmc->uk;
dmc->uk += dmc->deltaup[0];

// Przekaż dmc->uk do obiektu
// ...
```

Nie zarejestrowaliśmy odpowiedzi układu regulacji na skok wartości zadanej z powodu niewystarczającej ilości czasu na wykonanie zadania, natomiast regulator działał prawidłowo podczas pokazu działania urządzenia prowadzącemu laboratorium.

# Interfejs operatora

![Regulator w trakcie normalnej pracy\label{regnorm}](Zdjęcia do sprawka/IMG_0591.JPG){ width=75% }

Zaprezentowany na rysunku \ref{regnorm} interfejs operatora przedstawia domyślny tryb pracy urządzenia tzn. bez pojawienia się sytuacji nadzwyczajnych.

Konsekwentnie, kolor zielony symbolizuje wyjście obiektu, natomiast kolor niebieski -- sygnał sterujący.

Operator może kontrolować i analizować przebiegi angażując w to zadanie minimum wysiłku. W każdej chwili jest w stanie odczytać dokładny stan systemu. Dane są prezentowane największą możliwą czcionką pozwalającą na rozsądne rozmieszczenie reszty elementów. Dzięki tym zabiegom wyświetlacz jest również czytelny z dalszych odległości. Realizacja zmiany wartości zadanej została tak zaprojektowana, aby jak najbardziej ułatwić zadanie operatorowi.

## Kontekst

Kwadraty opisane G1, W1, T1 w sposób symboliczny przedstawiają obiekt.

## Wizualizacje

W prawym dolnym rogu jest prezentowany trend sygnału sterującego, wyjścia obiektu i wartości zadanej. Na osi pionowej zostały zaprezentowane dwie osobne skale dla tych wartości. Dla wyjścia wynosi ona od $30\;\text{\textdegree}C$ do $60\;\text{\textdegree}C$, a dla sterowania od $-50\;\%$ do $+50\;\%$.

W lewym dolnym rogu znajdują się dwa słupki, które prezentują aktualną wartość wyjścia obiektu i wartość sygnału sterującego. Ich skala jest zgodna z wykresem.

## Stan układu regulacji

Obok słupków prezentowana jest aktualna wartość wyjścia obiektu i sygnału sterującego. Wartość zadana pokazywana jest pod napisem ,,*Setpoint*''.

## Ingerencja operatora

Naciśnięcie przycisku "A/M" powoduje zmianę trybu pracy z automatycznego na ręczny i odwrotnie. W trybie automatycznym, pod napisem *Setpoint*, wyświetlana jest aktualna wartość zadana. W trybie ręcznym napis *Setpoint* zastępuje napis *Ctl. ef.*, a pod nim wyświetlana jest ustawiona wartość sygnału sterującego.

W prawym górnym rogu znajdują się przyciski służące do manipulacji opisanymi wyżej parametrami. Zostały one przemyślane następująco. Zmiana wartości o 1 w górę lub w dół jest realizowana za pomocą dużych przycisków, ponieważ łatwiejsze do naciśnięcia przyciski powinny realizować akcje mające mniejsze skutki aby zmniejszyć prawdopodobieństwo przypadkowego wykonania drastycznej akcji. Zakładamy również, że zmiany o małe wartości będą częstsze dzięki czemu duży przycisk ułatwi pracę. Mniejsze przyciski służą do zmiany o 10 w górę lub w dół. Zmiana ta będzie zgrubna czyli wiemy, że nie będzie to od razu oczekiwana dokładna wartość, dlatego następnie w dużej mierze operator będzie dobierał dokładniejszą wartość za pomocą dużych przycisków. Dobrana w ten sposób funkcjonalność przycisków miała na celu zapewnienie sprawniejszej interakcji z operatorem.

## Sytuacje awaryjne

### Błąd z komunikacją

Pod wpływem sugestii prowadzącego laboratorium zdecydowaliśmy, że komunikacja z obiektem jest bardzo istotna. Z tego powodu komunikat informujący o utracie połączenia powinien w dobitny sposób informować o problemie. Na naszym panelu taka sytuacja wygląda tak:

![Błąd komunikacji ze stanowiskiem](Zdjęcia do sprawka/IMG_0586.JPG){ width=75% }

Ekran staje się cały czerwony z białym komunikatem. Informacja o tej sytuacji nie znika nawet po przywróceniu połączenia. Dopiero ingerencja operatora (reset mikrokontrolera) powoduje zniknięcie komunikatu i powrót do normalnej pracy. Na zdjęciu widoczny jest również problem, który napotkaliśmy podczas implementacji ekranu błędu. Procedura realizacji przerwania przepełnienia jednego z liczników nie przestaje wypisywać na ekran wartości zadanej. Z powodu braku czasu, nie udało nam się tego naprawić. Nie jest to jednak dużym problemem, gdyż czerwony ekran jest dobrze widoczny z daleka i natychmiast przykuwa uwagę.

### Błąd czujnika temperatury

Uwzględniliśmy sytuację, w której dane odebrane z czujnika temperatury mogą być błędne tzn. odczytano temperaturę, która nie mieści się w zakresie mierzalnym przez czujnik. Taka sytuacja może okazać się bardzo groźna, ponieważ podczas jej występowania nie mamy informacji stanie obiektu. Gdy przekroczymy górne limity temperatur to może dojść do niebezpiecznej sytuacji. Jeżeli zejdziemy poniżej temperatury wymaganej do prawidłowej realizacji procesu możemy narazić właścicieli fabryki na duże straty.

Wiadomość o błędzie jest wyświetlana w lewym górnym prostokącie za pomocą dużej, wyraźnej, czerwonej czcionki.

![Błąd czujnika temperatury\label{probeerr}](Zdjęcia do sprawka/IMG_0592.JPG){ width=75% }

Napis *Temperature sensor error* zwraca uwagę operatora na fakt, iż nie można polegać na temperaturze odebranej z czujnika. Priorytet błędu uznaliśmy za wysoki, dlatego po przywróceniu komunikacji z czujnikiem operator musi ją zatwierdzić (reset mikrokontrolera) w celu usunięcia tego komunikatu. 

Podczas oglądania zdjęć zauważyliśmy problem, którego nie zauważyliśmy podczas testowania interfejsu. Aktualna wartość temperatury powinna być ukryta w sytuacji, gdy nie można na niej polegać. Pozwoliłoby to uniknąć problemu widocznego na zdjęciu \ref{tempwarn}, gdzie najmniej znacząca cyfra pochodzi z chwili, w której zostało wykonane zdjęcie \ref{probeerr}.

## Alarmy

Ostatnią obsługiwaną sytuacją nadzwyczajną jest zbyt duża lub zbyt mała temperatura (przekracza $60\;\text{\textdegree}C$ lub jest poniżej $30\;\text{\textdegree}C$). Ma to zapobiec nieprawidłowemu przebiegowi procesu. Wiadomość o takiej sytuacji pojawia się w prostokącie w lewym górnym rogu. Czerwony komunikat przekazuje klarowną informację o tym co dzieje się na stanowisku. Po powrocie temperatury do akceptowalnej wartości komunikat nie znika i wymaga potwierdzenia.

![Ostrzeżenie przed zbyt dużą lub zbyt małą temperaturą\label{tempwarn}](Zdjęcia do sprawka/IMG_0593.JPG){ width=75% }

# Wnioski

Zaprojektowanie dobrego interfejsu operatora nie jest łatwe i wymaga wiedzy z wielu dziedzin. Wymaga również czasu na testowanie i eliminację błędów. Nie dysponowaliśmy wystarczającą ilością czasu aby w sposób satysfakcjonujący wykonać powierzone nam zadanie, natomiast opuściliśmy laboratorium z działającym prototypem, który umożliwiał interakcję z obiektem.
