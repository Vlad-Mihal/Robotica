### Analisi Tecnica della Generazione RangeImage

## 1. Tipo di Modello Utilizzato

La classe `pcl::RangeImage` utilizza un modello di **Proiezione Sferica** (*Spherical Projection*).

### Come funziona
A differenza di una classica fotocamera (che usa una proiezione prospettica su un piano 2D piatto), la `RangeImage` simula un sensore che si trova esattamente al centro di una sfera.

### Coordinate
Ogni pixel dell'immagine non è identificato da (x,y) cartesiani, ma da coordinate polari/angolari:

* **Azimuth (phi):** L'angolo orizzontale (rotazione attorno all'asse Z).
* **Elevation (theta):** L'angolo verticale (inclinazione rispetto al piano).
* **Range (r):** La distanza misurata dal centro del sensore al punto colpito (il valore del pixel).

**Perché è funziona ?** Questo modello è matematicamente equivalente al funzionamento fisico di un LiDAR rotante (come il *Velodyne VLP-16*), che acquisisce dati ruotando ed emettendo raggi laser ad angoli fissi.

---

## 2. Influenza dei Parametri sulla Point Cloud

I parametri configurabili influenzano il risultato finale della simulazione in questo modo:

### Risoluzione Angolare (`res_x`, `res_y`)
Determina la densità dei punti.

* **Valori Alti (es. 1.0):** Creano una nuvola "sparsa", con molto spazio vuoto tra i punti (simile a un LiDAR reale a 16 o 32 linee).
* **Valori Bassi (es. 0.1):** Creano una nuvola "densa" e continua, simile a una fotografia 3D, utile per la visualizzazione umana ma computazionalmente più pesante.

### Field of View (FoV) (`fov_h`, `fov_v`)
Determina l'ampiezza della finestra di osservazione.

* **Riduzione FoV Verticale:** Se riduci `fov_v` (es. da 45° a 10°), vedrai solo una striscia sottile orizzontale (tagliando soffitto e pavimento).
* **Riduzione FoV Orizzontale:** Se riduci `fov_h` (es. da 360° a 90°), vedrai solo un "cono" davanti a te, invece dell'intera scena panoramica.

### Posa del Sensore (`x`, `y`, `z`)
Determina il centro di proiezione e, di conseguenza, le **occlusioni**.  
Spostando il sensore, cambiano le prospettive e ciò che è nascosto dietro gli ostacoli (es. i muri).

---

Lo stato di avanzamento del progetto è il seguente:

- [x] **Mapping:**  (Kiss-ICP + Open3D)
- [x] **Vista Virtuale:**  (PCL RangeImage)
- [x] **Parametrizzazione:**  (`rofl::ParamMap`)
- [x] **Confronto Viste:**  (file generati a 10m e 30m)
