# Projet STM32

Ce dépôt contient l'intégralité du code source et la documentation pour le projet STM32.

## Structure du projet

* **Racine** : Contient le projet STM32 complet (fichiers sources, includes, configuration).
* **datasheet/** : Contient la documentation technique générée (Doxygen/Datasheet).

---

## Documentation

La documentation se trouve dans le dossier `datasheet/`. Deux formats sont proposés, mais la **version HTML est fortement recommandée** pour la navigation.

### 1. Version HTML (Recommandée) 
Format le plus pratique pour explorer le code et les fonctions.

* **Emplacement :** Ouvrir le fichier `index.html` situé dans le dossier `datasheet/html/`.
* **⚠️ Attention à l'affichage :**
    > Si le texte est illisible ou apparaît "noir sur noir", cela est probablement dû à une extension de mode sombre (Dark Reader, Dark Mode natif, etc.).
    > **Désactiver le mode sombre du navigateur** pour consulter cette documentation correctement.

### 2. Version PDF
Une version PDF est normalement disponible à la racine du dossier `datasheet/`.

**Si le fichier PDF n'est pas présent**, il est nécessaire de le générer via LaTeX :

1.  Se rendre dans le dossier : `datasheet/latex/`
2.  Compiler le fichier source : `refman.tex`
3.  Le fichier généré sera disponible ici : `datasheet/latex/refman.pdf`

---

## Utilisation

1.  Cloner ce dépôt.
2.  Ouvrir le projet avec l'IDE habituel (ex: STM32CubeIDE).
3.  Consulter la documentation HTML dans `datasheet/html/` pour les détails d'implémentation.
