# UbxPvt

Lecture des messages UBX-NAV-PVT d'un récepteur u-blox (M8/M9, ex. NEO-M9N) sur un port
série matériel d'ESP32. La bibliothèque est volontairement simple et en **lecture seule** :

- **aucune tâche créée** : c'est le programme principal qui appelle `poll()` quand il le veut ;
- **non bloquante** : `poll()` vide juste le tampon RX et rend la main (microsecondes) ;
- **lecture seule** : rien n'est écrit vers le récepteur, la configuration se fait via
  u-center ou votre propre code ;
- **robuste** : machine à états incrémentale, re-synchronisation sur les octets de synchro
  `0xB5 0x62`, vérification Fletcher-8 (CKA/CKB) de chaque trame décodée ou remise au
  handler, re-calage immédiat sur une paire de synchro rencontrée dans une trame ignorée,
  pas d'allocation dynamique ;
- **extensible** : un handler optionnel reçoit toutes les trames valides (NAV-SAT, MON-VER,
  ACK, etc.), charge utile brute à vous de l'interpréter ;
- **testable** : le cœur (`UbxPvtParser`) est du C++ pur, sans aucune dépendance Arduino,
  couvert par des tests unitaires natifs (`pio test -e native`).

## Câblage (exemple M9N sur ESP32-S3)

| M9N | ESP32 |
| --- | --- |
| TX | GPIO2 (RX de l'UART1) |
| RX | GPIO3 (TX, non nécessaire en lecture seule) |
| GND | GND |

## Utilisation

```cpp
#include <UbxPvt.h>

HardwareSerial gpsSerial(1);
UbxPvt gps(gpsSerial);

void setup() {
  Serial.begin(115200);
  gps.begin(115200, /*rxPin=*/2, /*txPin=*/3); // baud du récepteur
}

void loop() {
  if (gps.poll()) {                 // true : un nouveau NAV-PVT vient d'être décodé
    const UbxNavPvt& pvt = gps.pvt();
    if (pvt.has3DFix()) {
      Serial.printf("lat %.7f lon %.7f alt %.1f m speed %.2f m/s, %02d:%02d:%02d UTC\n",
                    pvt.latitudeDeg(), pvt.longitudeDeg(), pvt.altitudeMsl(),
                    pvt.groundSpeedMps(), pvt.hour, pvt.minute, pvt.second);
    }
  }
}
```

Exemple de décodage d'autres trames via le handler générique :

```cpp
void onFrame(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t len, void* ctx) {
  if (msgClass == 0x01 && msgId == 0x35) { /* NAV-SAT : constellation */ }
}
gps.onFrame(onFrame);        // gps.onFrame(nullptr) pour désactiver
```

## API

### `UbxPvt` (wrapper Arduino, `src/UbxPvt.h`)

| Méthode | Description |
| --- | --- |
| `UbxPvt(HardwareSerial& serial)` | construit autour d'un port série matériel (ex. `Serial1`) |
| `begin(baud)` / `begin(baud, rxPin, txPin)` | configure le port ; sur ESP32 les broches sont routables |
| `poll()` | lit tout ce qui est disponible ; `true` si au moins un NAV-PVT décodé |
| `pvt()` | référence `const UbxNavPvt&` vers le dernier message décodé |
| `parser()` | accès au parser (statistiques) |
| `onFrame(handler, ctx = nullptr)` | handler appelé pour **chaque** trame valide (NAV-PVT inclus) |

### `UbxNavPvt` (données décodées, `src/UbxPvtParser.h`)

Champs bruts au format u-blox (`lon`/`lat` en 1e-7 degré, hauteurs en mm, vitesses en mm/s,
`heading` en 1e-5 degré, `pDOP` en 0.01, `iTOW` en ms, temps UTC `year`..`second`, `nano` en ns).
Accesseurs pratiques : `latitudeDeg()`, `longitudeDeg()`, `altitudeMsl()`, `groundSpeedMps()`,
`headingDeg()`, `hasFix()`, `has3DFix()`, `validDate()`, `validTime()`, `fullyResolved()`, `fixOk()`.

`fixType` : 0 = pas de fix, 2 = 2D, 3 = 3D, 4 = GNSS + dead-reckoning, 5 = heure seule.

### `UbxPvtParser` (cœur pur, réutilisable ailleurs)

`write(byte)` / `write(bytes, len)` → un NAV-PVT décodé ; `data()` → dernier décodé ;
`decodedCount()`, `checksumErrorCount()`, `skippedMessageCount()`, `syncCount()`, `reset()`.
Accepte les longueurs de charge utile NAV-PVT de 84 (protocole < 18) et 92 octets
(protocole ≥ 18, M8 firmware 2+, M9).

`onFrame(handler, ctx)` : le handler reçoit toutes les trames dont le checksum est valide,
NAV-PVT compris, avec `(class, id, payload, len, context)`. Trames plus longues que
`MaxFrameLen` (1024 octets) : consommées et ignorées. Le handler est appelé de façon
synchronisée depuis `write()` : le tampon de charge appartient au parser (à copier si
besoin), et il ne faut pas nourrir le parser depuis le callback.

## Tests

```sh
pio test -e native
```

19 tests : décodage champ par champ, validation checksum, re-synchronisation après
déchet/bruit/NMEA/trame tronquée, trames non-PVT entrelacées, décodage octet par octet,
longueurs 84/92, fuzz pseudo-aléatoire, handler générique (rappel, contexte, checksum
invalide, trame trop grande, trame vide, désenregistrement). La validation sur cible se
fait via `src/main.cpp` (balayage de baud, impression de la solution et décompte des types
de trames toutes les 5 s).

## Limites

- Seul NAV-PVT est décodé en champs ; les autres trames passent par `onFrame()` en brut.
- Lecture seule : ni configuration du récepteur, ni gestion des ACK.
- Sans `onFrame()`, les trames non-PVT sont ignorées sans vérification de checksum
  (seuls les NAV-PVT et les trames remises au handler sont validés).
