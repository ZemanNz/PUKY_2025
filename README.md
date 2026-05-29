# RBCX PUKY 2026 – Uživatelská Příručka & Dokumentace

Tento projekt obsahuje firmware pro autonomního robota PUKY, postaveného na platformě **RBCX** a ESP32. Tento model vychází z verze pro rok 2025, která byla výrazně vylepšena a optimalizována pro sezónu **2026**, díky čemuž robot dosahuje lepších výsledků a stability.

Hlavním úkolem robota je sběr, třídění a ukládání puků na hracím poli.

---

## 📋 Předstartovní Checklist

Před každým spuštěním robota na hrací ploše proveďte následující kontroly:

- [ ] **Připojení ultrazvuků**: Ověřte, že jsou všechny 4 ultrazvukové senzory pevně připojeny a správně zapojeny do příslušných konektorů.
- [ ] **Nabitá baterie**: Zkontrolujte napětí akumulátoru, zda je dostatečně nabitý pro celou jízdu.
- [ ] **Správně orientovaný třídič**: Ujistěte se, že třídicí klapka (chytré servo ID 0) je správně mechanicky namontovaná, volně se pohybuje a její výchozí poloha odpovídá zavřenému stavu.
- [ ] **Indikace při startu (žlutá LED)**: Po zapnutí/resetu robota se musí rozsvítit **žlutá LED dioda** (`rkLedYellow`). To signalizuje, že inicializace proběhla v pořádku a robot čeká v pohotovostním režimu na výběr strategie.
- [ ] **Očištění kol**: Před startem je doporučeno očistit kola vlhčeným ubrouskem (nebo podobným způsobem) pro odstranění prachu a zajištění maximální trakce.

---

## 🎮 Ovládání a Funkce Tlačítek

Robot se ovládá pomocí vestavěných tlačítek. Níže je popsáno chování robota po stisknutí jednotlivých tlačítek:

| Tlačítko | Název / Umístění | Akce / Popis |
| :--- | :--- | :--- |
| **`BTN_LEFT`** | **Zadní tlačítko** (Modrá strategie) | Spustí autonomní strategii **Modrá** (`modra()`). Zhasne žlutou LED, rozsvítí modrou LED (pozn. *modrá LED fyzicky nefunguje*), zavře všechna dvířka boxů a přepážku, aktivuje 3minutový bezpečnostní stopovač (`StopTask`) a po 1s prodlevě odstartuje jízdu. |
| **`BTN_RIGHT`** | **Přední tlačítko** (Červená strategie) | Spustí autonomní strategii **Červená** (`cervena()`). Zhasne žlutou LED, rozsvítí červenou LED, zavře všechna dvířka boxů a přepážku, aktivuje 3minutový bezpečnostní stopovač (`StopTask`) a po 1s prodlevě odstartuje jízdu. |
| **`BTN_ON`** | **Test Pohybu** | Spustí testovací sekvenci pohybu: jízda 1500 mm vpřed, otočení o 90° vpravo, otočení o 180° vlevo a couvání na zadní tlačítka (`back_buttons`). Na konci sekvence krátce pípne. |
| **`BTN_UP`** | **Test Hledání Základny** | Spustí testovací vyhledání červené základny `dojed_na_svoje(false)`. Pro účely testu virtuálně posune pozici robota na `[1000, 1000]`. |
| **`BTN_OFF`** | **Výpis Historie** | Vypíše kompletní uloženou historii měření barevných senzorů (`color_log_history`) do sériové linky. Úspěšné vypsání je indikováno dvojitým pípnutím bzučáku. |
| **`BTN_DOWN`** | **Komplexní Diagnostika** | Spustí sekvenční test všech hardwarových komponent: motorů (pohyb vpřed), zadních tlačítek, mechanických serv levé/pravé komory, chytrého serva třídiče, měření vzdálenosti na všech 4 ultrazvucích a čtení RGB hodnot ze senzorů `puky` a `zem`. |

> [!WARNING]
> **Modrá LED dioda** na robotu je nefunkční. Při spuštění modré strategie se programově aktivuje pin pro modrou LED, ale dioda se fyzicky nerozsvítí.

---

## ⚙️ Popis Třídicího Algoritmu a Boxů

Robot sbírá puky a pomocí barevného senzoru `puky` je detekuje a třídí pomocí chytrého serva (třídiče) do levého nebo pravého boxu:
- **Červené puky**: Jsou tříděny do **levého boxu** (`puk_do_l_boxu()`).
- **Modré puky**: Jsou tříděny do **pravého boxu** (`puk_do_r_boxu()`).

Třídicí úloha běží asynchronně ve vlastním FreeRTOS tasku (`ChytejPukyTask`) během jízdy robota vpřed. Kapacita každého boxu je omezena na `max_puck_per_box = 8`.

---

## 🛠️ Vývojářské Poznámky & Tipy

- **Bezpečnostní Stop (`StopTask`)**: Po spuštění libovolné strategie se spustí asynchronní task `StopTask`, který po přesně 3 minutách (180 000 ms) suspenduje hlavní vlákno, zastaví motory a začne střídavě blikat všemi LED diodami. Tím je zajištěno automatické ukončení zápasu a splnění pravidel soutěže.
