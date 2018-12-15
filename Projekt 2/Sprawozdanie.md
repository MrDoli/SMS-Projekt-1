SMS - Projekt 2

Krystian Chachuła, Marcin Dolicher

15 grudnia 2018



# Zagadnienia i założenia projektowe

Postawione przed nami zadanie polegało na implementacji algorytmu DMC do regulacji temperatury grzałki na stanowisku grzejąco - chłodzącym. Głównym celem tego projektu była implementacja interfejsu użytkownika regulatora oraz obsługa wszelkich możliwych stanów awaryjnych. W projekcie wykorzystaliśmy algorytm DMC napisany na potrzeby poprzedniego laboratorium. Dwa kluczowe stany awaryjne, które należy obsłużyć to błąd wykonania pomiaru i błąd komunikacji. Ochrona stanowiska przed przegrzaniem została wykonana sprzętowo. Wizualizacja miała zostać zrealizowana według założeń stosowanych przy projektowaniu paneli HMI tzn. miała być przede wszystkim czytelna i przekazywać wszystkie potrzebne informacje dla operatora. 

### Model regulatora i implementacja algorytmu DMC 

W celu wyznaczenia modelu regulatora użyliśmy odpowiedzi skokowej. Zmienialiśmy za pomocą skoku wartość zadaną dla regulatora i obserwowaliśmy jak zachowuje się wyjście regulatora. W naszym przypadku skok miał wartość 50. Z powodu bardzo małej dynamiki obiektu udało nam się przeprowadzić dwa eksperymenty. 

Następnie dokonaliśmy normalizację odpowiedzi skokowej i sygnału wartości zadanej. Wszystkie wartości z odpowiedzi pomniejszyliśmy o wartość odpowiedzi z pierwszej próbki dzięki czemu wartości odpowiedzi skokowej zaczynały się od 0.  Wartości sygnału skokowego podzieliliśmy przez 50 ???????. 

Z wykresu mogliśmy wyznaczyć wartość horyzontu dynamiki (jest on równy ilości próbek potrzebnych regulatorowi do ustabilizowania się wokół wartości zadanej). Ponieważ zadanie nie polegało na dobraniu najlepszych na stawów  dla regulatora, wartości horyzontu sterowania i predykcji przyjęliśmy równe horyzontowi dynamiki, a wartość lambdy równą jeden.  Wybrane przez nas parametry zapewniały satysfakcjonującą jakość regulacji.

Znając wartości horyzontów wyznaczyliśmy za pomocą skryptu <<<<<<<<<<udostępnionego przez Pana dr. Piotra Marusaka>>>>> potrzebne parametry do zaimplementowanego przez nas regulatora DMC w plikach Dmc.c i Dmc.h. W naszym regulatorze zastosowaliśmy ograniczenia na wartości sygnału sterującego tak żeby w żadnym z przypadków nie przekroczyło ono wartości -50 lub 50. Wszelkie wymagane obliczenia dla DMC są wykonywane w funkcji 

```c
float dmcCe(Dmc *dmc, float pv)
```



+++++++++++++++ WYKRESY ++++++++++++++++



# Interfejs użytkownika/operatora

+++++++++++++++++++Pytanie: Czy opisujemy implementację czy tylko wynik działania? +++++++++++++++

![IMG_0591](Zdjęcia do sprawka\IMG_0591.JPG)

Zaprezentowane powyżej GUI panelu operatora przedstawia domyślny tryb pracy urządzenia tzn. bez pojawienia się sytuacji alarmowych. 

### Omówienie poszczególnych elementów interfejsu

Górne lewe okno zarezerwowane jest do wyświetlania komunikatów alarmowych. Informacja o alarmie pojawi się w przypadku wystąpienia problemu z połączeniem do stanowiska lub przekroczeniem temperatury. Komunikat jest prezentowany w formie dużego, czerwonego, dobrze widocznego tekstu. Szczegółowe omówienie wyświetlania i obsługi sytuacji alarmowych są omówione w następnych punktach. 

Na prawo od okna o alarmach znajduje się miejsce na wyświetlanie wartości zadanej temperatury. Jej wartość jest pokazywana pod napisem "Set point". Poniżej tej wartości jest przycisk "A/M", którego naciśnięcie powoduje zmianę trybu pracy z automatycznego na manualny i odwrotnie. 

W prawym górnym rogu znajdują się przyciski do zwiększania wartości zadanej (Set point). Zostały one przemyślane następująco. Zmiana wartości o 1 w górę lub w dół jest realizowana za pomocą dużych przycisków, ponieważ chcieliśmy umożliwić operatorowi łatwiejsze dobieranie dokładnej wartości. Zakładamy, że zmiany o małe wartości będą częstsze dzięki czemu duży przycisk ułatwi mu pracę. Mniejsze przyciski służą do zmiany o 10 w górę lub w dół. Zmiana ta będzie zgrubna czyli wiemy, że nie będzie to od razu oczekiwana dokładna wartość, dlatego następnie w dużej mierze operator będzie dobierał dokładniejszą wartość za pomocą dużych przycisków. Dobrana w ten sposób funkcjonalność przycisków miała na celu zapewnienie lepszej interakcji z operatorem.  <------------ nie wiem czy zostawiamy to ostatnie zdanie, bez niego jest ślicznie 

W dolnym lewym rogu znajdują się dwa słupki, które prezentują graficznie aktualne wartości temperatury (kolor zielony) i wartości sterowania (kolor niebieski). Przy czym skala dla temperatury wynosi 30 C do 60 C, a sterowania -50 do 50. Zaraz obok słupków jest prezentowana liczbowo aktualna wartość temperatury (zielony) i sterowania (niebieski). Poniżej znajdują się oznaczenia elementów w instalacji grzejąco - chłodzącej, które wykorzystujemy w procesie (oznaczenia G1, W1, T1). 

W prawym dolnym rogu są prezentowane przebiegi sygnałów sterującego, aktualna wartość temperatury i wartość zadana temperatury.  Na osi OY zostały zaprezentowane dwie skale osobne skale dla tych wartości. 

### Ocena panelu

Prezentowana w ten sposób są czytelne na pierwszy rzut oka, co jest bardzo ważne przy projektowaniu paneli w systemach SCADA. Operator może kontrolować i analizować przebiegi angażując w to zadanie minimum wysiłku. W każdej chwili jest w stanie odczytać dokładny stan systemu. Dane są prezentowane największą możliwą czcionką pozwalającą na rozsądne rozmieszczenie reszty elementów. Dzięki tym zabiegom wyświetlacz jest również czytelny z dalszych odległości. Realizacja zmiany wartości zadanej została tak zaprojektowana, aby jak najbardziej ułatwić zadanie operatorowi.

# Obsługa sytuacji awaryjnych i alarmów

Dla naszego stanowiska przewidzieliśmy obsługę 3 sytuacji awaryjnych: utrata komunikacji z stanowiskiem, utrata danych z czujnika temperatury i za wysoka temperatura na grzałce. 

Doszliśmy do wniosku, że komunikacja z stanowiskiem i regulatorem jest dla nas rzeczą najważniejszą. Z tego powodu alarm informujący o utracie połącznia powinien dawać jasny komunikat i w dobitny sposób informować o problemie. Na naszym panelu taka sytuacja wygląda tak:

![IMG_0586](Zdjęcia do sprawka\IMG_0586.JPG)

Ekran staje się cały czerwony z białym komunikatem. Numer "55" informuje o numerze błędu. ???????????? Informacja o tej sytuacji jest na tyle poważna, że nawet po wykryciu przywrócenia połączenia informacja o błędzie nie znika. Dopiero ingerencja operatora i jego potwierdzenie o przywróceniu połączenia powoduje zniknięcie komunikatu i powrót do normalnej pracy. 



Następny alarm informuje o utracie danych przesyłanych z czujnika temperatury. Taka sytuacja może okazać się bardzo groźna, ponieważ tracimy informację o tym w jakiej sytuacji znajduję się nasz obiekt. Gdy przekroczymy górne limity temperatur to może dojść do niebezpiecznych sytuacji jak np. stopienie rdzenia w reaktorze jądrowym, albo wybuch grzałki lub gazów w cementowni. Jeżeli zejdziemy poniżej temperatury wymaganej do prawidłowej realizacji procesu narazimy właścicieli fabryki na duże straty i kary od organów kontrolnych. Z tych względów aktualna informacja o temperaturze jest nam bardzo potrzebna.

Wiadomość o błędzie jest wyświetlana w lewym górnym prostokącie za pomocą dużej, wyraźnej, czerwonej czcionki. ![IMG_0592](Zdjęcia do sprawka\IMG_0592.JPG)

Jak widzimy komunikat zajmuje 1/4 wyświetlacza co daje jasny sygnał dla operatora, że podczas działania urządzenia pojawił się problem. Oprócz napisu "Temperature sensor error" zmieniamy wyświetlaną wartość temperatury  na niemożliwą do osiągnięcia przez obiekt co ma dodatkowo przykuć uwagę personelu obsługującego obiekt. Poziom błędu uznaliśmy za wysoki, dlatego po przywróceniu komunikacji z czujnikiem operator musi ją zatwierdzić w celu usunięciu informacji o błędzie. 



Ostatnim obsługiwanym alarmem jest sygnał o zbyt dużej wartości temperatury na grzałce. Ma to zapobiec przegrzaniu stanowiska. Wiadomość o takiej sytuacji pojawia się w prostokącie w lewym górnym rogu. Czerwony komunikat przekazuje klarowną informację o tym co zadziało się na stanowisku. Po powrocie temperatury do akceptowalnej wartości komunikat znika co świadczy o prawidłowym funkcjonowaniu obiektu. ![IMG_0593](Zdjęcia do sprawka\IMG_0593.JPG)