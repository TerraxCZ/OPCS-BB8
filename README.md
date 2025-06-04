# 2D Ballbot – Modelování a řízení pomocí LQR a MPC

Tento semestrální projekt se zabývá modelováním a řízením balancujícího robota (tzv. *ballbot*), který se pohybuje ve 2D rovině po kouli. Cílem je navrhnout řízení schopné stabilizovat systém pomocí metod **LQR** (Lineární kvadratický regulátor) a **MPC** (Prediktivní řízení) na základě:

- odvozeného nelineárního modelu pomocí Lagrangeových rovnic,
- linearizace v okolí rovnovážného bodu,
- implementace řízení v jazycích MATLAB a Python (OSQP solver).

## Obsah

- [Popis systému](#popis-systému)
- [Matematický model](#matematický-model)
- [Linearizace a diskrétní model](#linearizace-a-diskrétní-model)
- [LQR řízení (MATLAB)](#lqr-řízení-matlab)
- [MPC řízení (Python)](#mpc-řízení-python)
- [Simulace a výsledky](#simulace-a-výsledky)
- [Závěr](#závěr)

---

## Popis systému

Model systému byl inspirován droidem BB8 ze StarWars.  
Je to nestabilní dynamický systém složený z:

- **duté koule** (balón), která se může otáčet a tím se i posouvat po podložce,
- **těla robota**, které balancuje na kouli.

Pohyb je omezen pouze na jednu rovinu, čímž vzniká 2D model, kde je řízen pouze jeden stupeň volnosti pohybu po podložce a jeden stupeň volnosti náklonu těla.

Akční člen je realizován jako **moment působící na kouli** kolem horizontální osy. Tento moment způsobuje její otáčení a tím i posun po zemi. Cílem řízení je udržet těžiště robota nad koulí, tedy stabilizovat vertikální polohu těla.

### Ilustrační obrázek robota

<img src="Images/BB8_real.jpeg" alt="Ilustrační obrázek" width="600"/>

### Schéma systému

<img src="Images/BallBOt_diagram.svg" alt="Schéma systému" width="600"/>

## Matematický model

### Tabulka konstant

| Symbol           | Význam                                 | Hodnota  | Jednotka |
|------------------|----------------------------------------|----------|----------|
| m<sub>b</sub>    | hmotnost koule                         | 2.5      | kg       |
| m<sub>t</sub>    | hmotnost těla                          | 1.0      | kg       |
| J<sub>b</sub>    | moment setrvačnosti koule              | 0.085    | kg·m²    |
| R                | poloměr koule                          | 0.16     | m        |
| l                | vzdálenost těžiště těla od středu koule| 0.6      | m        |
| g                | gravitační zrychlení                   | 9.81     | m/s²     |

### Stavové veličiny

> **Poznámka:** Stavová veličina $x_1$ (poloha koule) je v modelu přepočítána z úhlu natočení koule $\theta$ podle vztahu $x = R_b \cdot \theta$.

| Symbol            | Stavová veličina | Význam                   | Jednotka |
|-------------------|------------------|--------------------------|----------|
| $x(t)$               | $x_1$            | poloha koule             | m        |
| $\dot{x}(t)$         | $x_2$            | rychlost koule           | m/s      |
| $\varphi(t)$         | $x_3$            | úhel náklonu těla        | rad      |
| $\dot{\varphi}(t)$   | $x_4$            | úhlová rychlost těla     | rad/s    |

---

### Poloha a rychlost středu koule

> **Poznámka:** Pro přehlednost není v následujících rovnicích explicitně uváděna závislost na čase, tedy např. místo $x(t)$ je psáno pouze $x$.  
> Například: $x = R \cdot \theta$ znamená $x(t) = R \cdot \theta(t)$.

- **Poloha středu koule:**  
$x = R \cdot \theta$  
$y = R$

- **Rychlost středu koule:**  
$\dot{x} = R \cdot \dot{\theta}$  
$\dot{y} = 0$

### Poloha a rychlost středu těla robota

**Poloha středu těla:**  
$x_{t} = x + l \cdot \sin(\varphi)$  
$y_{t} = l \cdot \cos(\varphi)$

**Rychlost středu těla:**  
$\dot{x}_{t} = \dot{x} + l \cdot \cos(\varphi) \cdot \dot{\varphi}$  
$\dot{y}_{t} = -l \cdot \sin(\varphi) \cdot \dot{\varphi}$

---

*Poznámka: Rovnice budou doplněny podle zápisu v MATLABu.*