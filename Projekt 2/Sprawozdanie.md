SMS - Projekt 2

Krystian Chachuła, Marcin Dolicher

15 grudnia 2018

# Cel

Postawione przed nami zadanie polegało na implementacji algorytmu DMC do regulacji temperatury na stanowisku grzejąco-chłodzącym. Głównym celem tego projektu była implementacja interfejsu użytkownika układu regulacji oraz obsługa wszelkich możliwych stanów awaryjnych. W projekcie wykorzystaliśmy algorytm DMC napisany na potrzeby poprzedniego laboratorium. Dwa kluczowe stany awaryjne, które należy obsłużyć to błąd wykonania pomiaru i błąd komunikacji. Ochrona stanowiska przed przegrzaniem została wykonana sprzętowo. Wizualizacja miała zostać zrealizowana według założeń stosowanych przy projektowaniu paneli HMI tzn. miała być przede wszystkim czytelna i przekazywać wszystkie potrzebne informacje dla operatora. 

### Model obiektu i implementacja algorytmu DMC 

W celu wyznaczenia modelu obiektu użyliśmy jego odpowiedzi skokowej. Aby ją pozyskać, w stanie ustalonym, zwiększyliśmy wartość sygnału sterującego obiektem z $-20$ na $30$ i obserwowaliśmy jak zachowuje się jego wyjście. Próbki wyjścia obiektu zachowaliśmy w celu dalszej analizy. Z powodu wolnej dynamiki obiektu udało nam się przeprowadzić tylko dwa eksperymenty.

Następnie dokonaliśmy normalizacji odpowiedzi skokowej. Wszystkie próbki wyjścia procesu pomniejszyliśmy o wartość pierwszej z nich. Aby zakończyć normalizację, podzieliliśmy wszystkie próbki przez $50$, ponieważ o tyle zwiększyliśmy wartość sygnału sterującego podczas eksperymentu.

Z wykresu odczytaliśmy horyzont dynamiki obiektu tzn. liczbę kwantów czasu (próbek) potrzebnych obiektowi do ustabilizowania się po skoku sygnału sterującego. Ponieważ tym razem zadanie nie polegało na dobraniu najlepszych na stawów dla regulatora, wartości horyzontu sterowania i predykcji przyjęliśmy równe horyzontowi dynamiki, a wartość kary za zmiany sygnału sterującego równą jeden. Wybrane przez nas parametry zapewniały satysfakcjonującą jakość regulacji.

Naszym kolejnym krokiem było wyznaczenie współczynników prawa regulacji za pomocą skryptu udostępnionego przez dr. Piotra Marusaka. W naszym regulatorze zastosowaliśmy ograniczenia na wartości sygnału sterującego tak żeby w żadnym z przypadków nie przekroczyło ono wartości -50 lub 50. Wszelkie wymagane obliczenia dla DMC są wykonywane w funkcji 

```c
float dmcCe(Dmc *dmc, float pv)
```

[//1]: # (Wykresy)

# Interfejs użytkownika/operatora

[//2]: # (Czy opisujemy implementację czy tylko wynik działania?)

![IMG_0591](Zdjęcia do sprawka\IMG_0591.JPG)

Zaprezentowane powyżej GUI panelu operatora przedstawia domyślny tryb pracy urządzenia tzn. bez pojawienia się sytuacji alarmowych.

### Omówienie poszczególnych elementów interfejsu

Górne lewe okno zarezerwowane jest do wyświetlania komunikatów alarmowych. Informacja o alarmie pojawi się w przypadku wystąpienia problemu z połączeniem do stanowiska lub przekroczeniem temperatury. Komunikat jest prezentowany w formie dużego, czerwonego, dobrze widocznego tekstu. Szczegółowe omówienie wyświetlania i obsługi sytuacji alarmowych są omówione w następnych punktach. 

Na prawo od okna o alarmach znajduje się miejsce na wyświetlanie wartości zadanej temperatury. Jej wartość jest pokazywana pod napisem ,,Setpoint''. Poniżej tej wartości jest przycisk "A/M", którego naciśnięcie powoduje zmianę trybu pracy z automatycznego na manualny i odwrotnie. 

W prawym górnym rogu znajdują się przyciski do zwiększania wartości zadanej (Setpoint). Zostały one przemyślane następująco. Zmiana wartości o 1 w górę lub w dół jest realizowana za pomocą dużych przycisków, ponieważ chcieliśmy umożliwić operatorowi łatwiejsze dobieranie dokładnej wartości. Zakładamy, że zmiany o małe wartości będą częstsze dzięki czemu duży przycisk ułatwi mu pracę. Mniejsze przyciski służą do zmiany o 10 w górę lub w dół. Zmiana ta będzie zgrubna czyli wiemy, że nie będzie to od razu oczekiwana dokładna wartość, dlatego następnie w dużej mierze operator będzie dobierał dokładniejszą wartość za pomocą dużych przycisków. Dobrana w ten sposób funkcjonalność przycisków miała na celu zapewnienie lepszej interakcji z operatorem.

W lewym dolnym rogu znajdują się dwa słupki, które prezentują graficznie aktualne wartości temperatury (kolor zielony) i wartości sterowania (kolor niebieski). Przy czym skala dla temperatury wynosi 30 C do 60 C, a sterowania -50 do 50. Zaraz obok słupków jest prezentowana liczbowo aktualna wartość temperatury (zielony) i sterowania (niebieski). Poniżej znajdują się oznaczenia elementów układu regulacji (oznaczenia G1, W1, T1). 

W prawym dolnym rogu są prezentowane przebiegi sygnałów sterującego, aktualna wartość temperatury i wartość zadana temperatury. Na osi pionowej zostały zaprezentowane dwie osobne skale dla tych wartości. 

### Ocena panelu

Prezentowana w ten sposób są czytelne na pierwszy rzut oka, co jest bardzo ważne przy projektowaniu paneli w systemach SCADA. Operator może kontrolować i analizować przebiegi angażując w to zadanie minimum wysiłku. W każdej chwili jest w stanie odczytać dokładny stan systemu. Dane są prezentowane największą możliwą czcionką pozwalającą na rozsądne rozmieszczenie reszty elementów. Dzięki tym zabiegom wyświetlacz jest również czytelny z dalszych odległości. Realizacja zmiany wartości zadanej została tak zaprojektowana, aby jak najbardziej ułatwić zadanie operatorowi.

# Obsługa sytuacji awaryjnych i alarmów

Dla naszego stanowiska przewidzieliśmy obsługę 3 sytuacji awaryjnych: utrata komunikacji z stanowiskiem, utrata danych z czujnika temperatury i za wysoka temperatura na grzałce. 

Doszliśmy do wniosku, że komunikacja z stanowiskiem i regulatorem jest dla nas rzeczą najważniejszą. Z tego powodu alarm informujący o utracie połączenia powinien dawać jasny komunikat i w dobitny sposób informować o problemie. Na naszym panelu taka sytuacja wygląda tak:

![IMG_0586](Zdjęcia do sprawka\IMG_0586.JPG)

Ekran staje się cały czerwony z białym komunikatem. Informacja o tej sytuacji nie znika nawet po przywróceniu połączenia. Dopiero ingerencja operatora (reset mikrokontrolera) powoduje zniknięcie komunikatu i powrót do normalnej pracy. Na zdjęciu widoczny jest również problem, który napotkaliśmy podczas implementacji ekranu błędu. Procedura realizacji przerwania przepełnienia jednego z liczników nie przestaje wypisywać na ekran wartości zadanej. Z powodu braku czasu, nie udało nam się tego naprawić.

Następny alarm informuje o odebraniu błędnych danych z czujnika temperatury. Taka sytuacja może okazać się bardzo groźna, ponieważ nie mamy informacji stanie obiektu. Gdy przekroczymy górne limity temperatur to może dojść do niebezpiecznych sytuacji jak np. stopienie rdzenia w reaktorze jądrowym, albo uszkodzenie grzałki lub eksplozji gazów w cementowni. Jeżeli zejdziemy poniżej temperatury wymaganej do prawidłowej realizacji procesu narazimy właścicieli fabryki na duże straty i kary od organów kontrolnych. Z tych względów aktualna informacja o temperaturze jest nam bardzo potrzebna.

Wiadomość o błędzie jest wyświetlana w lewym górnym prostokącie za pomocą dużej, wyraźnej, czerwonej czcionki.

![IMG_0592](Zdjęcia do sprawka\IMG_0592.JPG)

Jak widzimy komunikat zajmuje 1/4 wyświetlacza co daje jasny sygnał dla operatora, że podczas działania urządzenia pojawił się problem. Napis ,,Temperature sensor error'' zwraca uwagę operatora na fakt, iż nie można polegać na temperaturze odebranej z czujnika. Poziom błędu uznaliśmy za wysoki, dlatego po przywróceniu komunikacji z czujnikiem operator musi ją zatwierdzić (reset mikrokontrolera) w celu usunięciu informacji o błędzie. 

Ostatnim obsługiwanym alarmem jest sygnał o zbyt dużej lub zbyt małej temperaturze na grzałce. Ma to zapobiec nieprawidłowemu przebiegowi procesu. Wiadomość o takiej sytuacji pojawia się w prostokącie w lewym górnym rogu. Czerwony komunikat przekazuje klarowną informację o tym co zadziało się na stanowisku. Po powrocie temperatury do akceptowalnej wartości komunikat znika co świadczy o prawidłowym funkcjonowaniu obiektu.

![IMG_0593](Zdjęcia do sprawka\IMG_0593.JPG)