/**
 * @file pixy2.cpp
 * @brief file containing the pixy2 class programs, compatible with mbed-os 6 (baremetal or multithread) using UART interface with non blocking functions
 * @author Hugues Angelis - Théo Le Paih - Wael Hazami
 * @date March 2022
 */
 
#include "pixy2.h"

int sommeDeControle,sommeRecue;

PIXY2::PIXY2(PinName tx, PinName rx, int debit) : Pixy2_numBlocks(0), Pixy2_numVectors(0), Pixy2_numIntersections(0), Pixy2_numBarcodes(0)
{   
    _Pixy2 = new UnbufferedSerial (tx, rx, debit);
    _Pixy2->attach (callback(this,&PIXY2::pixy2_getByte));
    etat = idle;
    Pixy2_buffer = (Byte*) malloc (0x100); 
}

PIXY2::~PIXY2()
{
    free (Pixy2_buffer);
}

// POUR DEBUG //
PIXY2::T_Pixy2State PIXY2::getEtat()
{
    return this->etat;
}

void PIXY2::affDataSize()
{
    printf("dataSize : %d\n\r", this->dataSize);
}
// Pour détail d'une trame (a mettre dans la fct correspondante) :
//pc.printf("pixSync : %d, pixType : %d, pixLength : %d, pixChecksum : %d\n\r", msg->pixSync, msg->pixType, msg->pixLength, msg->pixChecksum);

/* Le programme utilise l'interruption de la liaison série pour recevoir le message et avancer la machine d'état pour permettre au programme de ne pas être bloquant en réception.
   Lorsqu'on appelle une fonction, elle retourne le code -2 pour signifier que le message de réponse de la caméra n'est pas completement reçu...
   Les étapes de fonctionnement sont les suivantes :
   
   * idle : La caméra n'a pas été solicité => on peut envoyer le message de requête.
   * messageSent : La requête a été transmise, mais la réponse n'est pas encore arrivée.
   * receivingHeader : Le mot de synchro a été reçu, le reste de l'entête est en cours de réception.
   * receivingData : L'entête a été intégralement reçu (et traité) et on est en train de recevoir la payload.
   * dataReceived : La réponse a été intégralement reçue et est disponible pour le traitement (et la libération).
   
   On utilise plusieurs pointeurs : 
   * wPointer est le pointeur d'écriture des octets reçus dans le buffer de réception
   * hPointer est le pointeur de l'entête du message
   * dPointer est le pointeur de la zone de données
   
   On rajoute une variable : dataSize, qui définit la quantité de données à recevoir (issue de l'entête)
   
   **** COMMENT CA MARCHE ****
   
   Au début wPointer = 0. Quand wPointer est supérieur à 1 (donc qu'on a reçu au moins 2 octets), on regarde si on a reçu le code de début de trame 0xC1AF
   Si ce n'est pas le cas, on continue de recevoir les octets et on les stock jusqu'à trouver le code de début de trame.
   Quand on trouve le code, on enregistre le point de départ (c'est hPointer) comme étant la position actuelle du wPointer-1.
   A partir de ça, on attend de recevoir les 6 premiers octets de l'entête :
   * start of frame (hPointer et hPointer + 1)
   * identification du type (hPointer + 2)
   * taille des données (hPointer + 3)
   Et ensuite si on a une trame avec un checksum (normal) :
   * le checksum (hPointer + 4 et hPointer + 5)
   * les données (hPointer + 6 et suivant)
   Et si on a une trame sans checksum (pas normal) :
   * les données (hPointer + 4 et suivant)
   
   On continue alors de recevoir des octets jusqu'à ce que wPointer soit égal à dPointer + dataSize - 1
   par exemple si la data fait 4 octets et une trame avec checksum, on reçoit tant que wPointer est inférieur à 9 (6 + 4 - 1)
   ou pour la même quantité de données mais avec une trame sans checksum 7 (4 + 4 - 1)...
   
                                          Automate des fonctions.

         appel de la fonction    
   /---        publique      ---\ /-------------------- géré par l'interruption -------------------\

    |------|  envoi   |---------|  start  |-----------|  header  |-----------|  trame  |----------|
    |      |  ordre   |         |  reçu   | receiving |   reçu   | receiving |  reçue  |   data   |
    | Idle |----+---->| msgSent |----+--->|   header  |----+---->|    data   |----+--->| received |---\
    |      |          |         |   C1AF  |           |    6o    |           | (data   |          |   |
    |------|          |---------|(ou C1AE)|-----------|          |-----------|  size)  |----------|   |
       |                                                                                              |
       \-------------------<---------------------------+---------------------------<------------------/
                                          lecture des données reçues
                                                         
       \-------------------------------  appel de la fonction publique  ------------------------------/

    Pour l'utilisateur seul l'appel de la fonction publique est nécessaire.
    Tant qu'il récupère un code de retour -1, cela signifie que la tâche n'est pas achevée
    Quand le code reçu est 0, cela signifie que le résultat est disponible
    Toutes les autres valeurs signifient une erreur
*/

void PIXY2::pixy2_getByte ()    // Interruption de la pixy2
{
    T_Word                  *buffer;
    
    _Pixy2->read(&Pixy2_buffer[wPointer],1);                                        // On stocke l'octet reçu dans la première case dispo du buffer de réception
    
    
    switch (etat) {
        case messageSent :                                                          // Si on a envoyé une requete => on attend un entête
            if (wPointer > 0) {                                                     // On attend d'avoir reçu 2 octets
                buffer = (T_Word*) &Pixy2_buffer[wPointer-1];                       // On pointe la structure sur les 2 derniers octets reçus
                if ((buffer->mot == PIXY2_CSSYNC) || (buffer->mot == PIXY2_SYNC)) { // Si c'est un mot d'entête
                    etat = receivingHeader;                                         // On passe à l'état réception de l'entête
                    hPointer = wPointer - 1;                                        // On initialise le pointeur de l'entête
                    if (buffer->mot == PIXY2_SYNC) {
                        frameContainChecksum = 0;                                   // Si c'est un entête sans checksum, on mémorise qu'il n'y a pas de checksum à vérifier
                        dPointer = hPointer + PIXY2_NCSHEADERSIZE;
                    } else {
                        frameContainChecksum = 1;                                   // Sinon, on mémorise qu'il y a un checksum à vérifier
                        dPointer = hPointer + PIXY2_CSHEADERSIZE;
                    }
                }                                                                   // Si on n'a pas de mot d'entête on attend d'en trouver un...
            }
            break;

        case receivingHeader :                                                      // Si on est en train de recevoir un entête (entre le SYNC et... La fin de l'entête)
            if ((frameContainChecksum && ((wPointer - hPointer) == (PIXY2_CSHEADERSIZE - 1))) || (!frameContainChecksum && ((wPointer - hPointer) == (PIXY2_NCSHEADERSIZE - 1)))) {
                                                                                    // Si on a reçu 6 octets pour une trame avec checksum ou 4 pour une trame sans checksum, c'est à dire un entête complet
                etat = receivingData;                                               // On dit que l'on va de recevoir des données
                dataSize = Pixy2_buffer[hPointer + 3];                              // On enregistre la taille de la payload
                if (dataSize == 0)                                                  // Si on ne doit recevoir qu'un entête, on a terminé
                    etat = idle;                                                    // On revient à l'état d'attente d'ordre
            }    
            break;

        case receivingData :                                                        // Si on est en train de recevoir des données.
            if (wPointer == ((dataSize - 1) + dPointer)) {                          // Quand on a reçu toutes les données
                etat = dataReceived;                                                // On dit que c'est OK pour leur traitement         
            }
            break;

        default : // On ne traite volontairement ici pas tous les cas, en particulier idle et dataReceived. C'est à la fonction de le faire ! Le reste est lié à des réceptions de données.
            break;
    }
    wPointer++;                                                                     // on pointe la case suivante du buffer de réception

}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetVersion (void){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 0;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_VERS;
    msg.frame.header.pixLength = dataSize;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetResolution (void){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 1;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_RESOL;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = 0;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetCameraBrightness (Byte brightness){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 1;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_BRIGHT;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = brightness;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetServo (Word s0, Word s1){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 4;
    T_Word              tmp;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_SERVOS;
    msg.frame.header.pixLength = dataSize;
    tmp.mot = s0;
    msg.frame.data[0] = tmp.octet[0];
    msg.frame.data[1] = tmp.octet[1];
    tmp.mot = s1;
    msg.frame.data[2] = tmp.octet[0];
    msg.frame.data[3] = tmp.octet[1];
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetLED (Byte red, Byte green, Byte blue){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 3;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_LED;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = red;
    msg.frame.data[1] = green;
    msg.frame.data[2] = blue;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetLamp (Byte upper, Byte lower){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 2;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_LAMP;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = upper;
    msg.frame.data[1] = lower;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetFPS (void){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 0;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_FPS;
    msg.frame.header.pixLength = dataSize;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetBlocks (Byte sigmap, Byte maxBloc){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 2;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_BLOC;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = sigmap;
    msg.frame.data[1] = maxBloc;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetLineFeature (Byte type, Byte feature){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 2;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_LINE;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = type;
    msg.frame.data[1] = feature;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetMode (Byte mode){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 1;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_MODE;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = mode;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetNextTurn (Word angle){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 2;
    T_Word              tmp;
    tmp.mot = angle;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_TURN;
    msg.frame.header.pixLength = dataSize;
    tmp.mot = angle;
    msg.frame.data[0] = tmp.octet[0];
    msg.frame.data[1] = tmp.octet[1];
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetDefaultTurn (Word angle){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 2;
    T_Word              tmp;
    tmp.mot = angle;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_DEFTURN;
    msg.frame.header.pixLength = dataSize;
    tmp.mot = angle;
    msg.frame.data[0] = tmp.octet[0];
    msg.frame.data[1] = tmp.octet[1];
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndSetVector (Byte vectorIndex){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 1;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_VECTOR;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = vectorIndex;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndReverseVector (void){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 0;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_SET_REVERSE;
    msg.frame.header.pixLength = dataSize;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_sndGetRGB (Word x, Word y, Byte saturate){
    T_pixy2SendBuffer   msg;
    int                 i = 0, dataSize = 5;
    msg.frame.header.pixSync = PIXY2_SYNC;
    msg.frame.header.pixType = PIXY2_ASK_VIDEO;
    msg.frame.header.pixLength = dataSize;
    msg.frame.data[0] = x;
    msg.frame.data[1] = y;
    msg.frame.data[2] = saturate;
    do {
        while(!_Pixy2->writable());
        _Pixy2->write(&msg.data[i],1);
        i++;
    } while (i<(PIXY2_NCSHEADERSIZE+dataSize));
    return PIXY2_OK;
}

/*  La fonction est bloquante à l'envoi (pas vraiment le choix), mais elle est non bloquante en réception. On essayera de faire une fonction non bloquante en envoi avec write, mais c'est pas la priorité.
Le principe c'est de stocker dans un buffer circulaire les données au fur et à mesure qu'elle sont reçues et de traiter uniquement en castant les infos. Pour cela, il faut recevoir et stocker. */

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getVersion (T_pixy2Version **ptrVersion){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetVersion();                                      // On envoie la trame de demande de la version
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if (msg->pixType == PIXY2_REP_VERS) {                                   // On vérifie que la trame est du type convenable (REPONSE VERSION)
                *ptrVersion =   (T_pixy2Version*) &Pixy2_buffer[dPointer];          // On mappe le pointeur de structure sur le buffer de réception.         
            } else {                                                                // Si ce n'est pas le bon type
                if (msg->pixType == PIXY2_REP_ERROR) {                              // Cela pourrait être une trame d'erreur
                    cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];              // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
                } else cr = PIXY2_TYPE_ERROR;                                       // Si le type ne correspond à rien de normal on signale une erreur de type.
            }
            etat = idle;                                                            // On annonce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getResolution (T_pixy2Resolution **ptrResolution){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetResolution();                                   // On envoie la trame de demande de la résolution
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if (msg->pixType == PIXY2_REP_RESOL) {                                  // On vérifie que la trame est du type convenable (REPONSE RESOLUTION)
                *ptrResolution = (T_pixy2Resolution*) &Pixy2_buffer[dPointer];      // On mappe le pointeur de structure sur le buffer de réception.
            } else {                                                                // Si ce n'est pas le bon type
                if (msg->pixType == PIXY2_REP_ERROR) {                              // Cela pourrait être une trame d'erreur
                    cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];              // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
                } else cr = PIXY2_TYPE_ERROR;                                       // Si le type ne correspond à rien de normal on signale une erreur de type.
            }
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setCameraBrightness (Byte brightness){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetCameraBrightness (brightness);                  // On envoie la trame de règlage de la luminosité
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setServos (Word s0, Word s1){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetServo (s0, s1);                                 // On envoie la trame de règlage des servos moteurs
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setLED (Byte red, Byte green, Byte blue){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetLED (red, green, blue);                         // On envoie la trame de règlage des composantes de la LED RGB
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setLamp (Byte upper, Byte lower){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetLamp (upper, lower);                            // On envoie la trame de règlage d'allumage des lumières de contraste
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getFPS (T_pixy2ReturnCode **framerate){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetFPS();                                          // On envoie la trame de demande du Framerate
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if (msg->pixType == PIXY2_REP_FPS) {                                    // On vérifie que la trame est du type convenable (REPONSE FPS)
                *framerate = (T_pixy2ReturnCode*) &Pixy2_buffer[dPointer];           // On mappe le pointeur de structure sur le buffer de réception.
            } else {                                                                // Si ce n'est pas le bon type
                if (msg->pixType == PIXY2_REP_ERROR) {                              // Cela pourrait être une trame d'erreur
                    cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];              // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
                } else cr = PIXY2_TYPE_ERROR;                                       // Si le type ne correspond à rien de normal on signale une erreur de type.
            }
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getBlocks (Byte sigmap, Byte maxBloc){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetBlocks(sigmap, maxBloc);                        // On envoie la trame de demande de blocs de couleur
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if (msg->pixType == PIXY2_REP_BLOC) {                                   // On vérifie que la trame est du type convenable (REPONSE BLOCS)
                Pixy2_blocks = (T_pixy2Bloc*) &Pixy2_buffer[dPointer];              // On mappe le pointeur de structure sur le buffer de réception.
                Pixy2_numBlocks = dataSize / sizeof(T_pixy2Bloc);                   // On indique le nombre de blocs reçus
            } else {                                                                // Si ce n'est pas le bon type
                if (msg->pixType == PIXY2_REP_ERROR) {                              // Cela pourrait être une trame d'erreur
                    cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];              // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
                } else cr = PIXY2_TYPE_ERROR;                                       // Si le type ne correspond à rien de normal on signale une erreur de type.
            }
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getFeatures (){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    T_pixy2LineFeature* lineFeature;
    int                 fPointer;                                                   // Pointeur sur une feature entière
    int                 fdPointer;                                                  // Pointeur sur un élément à l'intérieur d'une feature

    if (frameContainChecksum) {                                                     // Si la trame contient un checksum
        if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {                // On lance la validation du checksum
            return PIXY2_BAD_CHECKSUM;                                              // Si le checksum est faux on retourne une erreur
        }
    }
    if (msg->pixType == PIXY2_REP_LINE) {                                       // On vérifie que la trame est du type convenable (REPONSE LIGNE)
        fPointer = dPointer;                                                        // On pointe sur la premiere feature
        do {
            lineFeature = (T_pixy2LineFeature*) &Pixy2_buffer[fPointer];            // On mappe le pointeur de structure sur le buffer de réception des features.
            if (lineFeature->fType == PIXY2_VECTOR) {                               // On regarde si le type est vecteur
                Pixy2_numVectors = lineFeature->fLength / sizeof(T_pixy2Vector);    // Si oui, on compte combien il y a de vecteurs
                fdPointer = fPointer + 2;                                           // On pointe sur le premier élément de la feature
                Pixy2_vectors = (T_pixy2Vector*) &Pixy2_buffer[fdPointer];          // On mappe le résultat
                fPointer += lineFeature->fLength + 2;                               // On déplace le pointeur de données et on recommence
                cr |= PIXY2_VECTOR;
            }
            if (lineFeature->fType == PIXY2_INTERSECTION) {                         // On regarde si le type est intersection
                Pixy2_numIntersections = lineFeature->fLength / sizeof(T_pixy2Intersection);
                                                                                    // Si oui, on compte combien il y a d'intersections
                fdPointer = fPointer + 2;                                           // On pointe sur le premier élément de la feature                                                                    
                Pixy2_intersections = (T_pixy2Intersection*) &Pixy2_buffer[fdPointer];
                                                                                    // On mappe le résultat sur l'entête de l'intersection
                fPointer += lineFeature->fLength + 2;                               // On déplace le pointeur de données et on recommence
                cr |= PIXY2_INTERSECTION;
            }
            if (lineFeature->fType == PIXY2_BARCODE) {                              // On regarde si le type est codebarre
                Pixy2_numBarcodes = lineFeature->fLength / sizeof(T_pixy2BarCode);
                                                                                    // Si oui, on compte combien il y a de codebarre
                fdPointer = fPointer + 2;                                           // On pointe sur le premier élément de la feature
                Pixy2_barcodes = (T_pixy2BarCode*) &Pixy2_buffer[fdPointer];        // On mappe le résultat
                fPointer += lineFeature->fLength + 2;                               // On déplace le pointeur de données et on recommence
                cr |= PIXY2_BARCODE;
            }
        } while(fPointer < ((dataSize - 1) + dPointer));                            // Tant qu'il y a des données à traiter
    } else {                                                                        // Si ce n'est pas le bon type
        if (msg->pixType == PIXY2_REP_ERROR) {                                      // Cela pourrait être une trame d'erreur ou quand on ne reçoit rien
            cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                      // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
        } else cr = PIXY2_TYPE_ERROR;                                               // Si le type ne correspond à rien de normal on signale une erreur de type.
    }
    etat = idle;                                                                    // On annoce que la pixy est libre
    return cr;
}


PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getMainFeature (Byte features){

    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetLineFeature(0, features);                      // On envoie la trame de demande de suivi de ligne
            if (cr!= PIXY2_OK) return cr;                                          // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                    // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            cr = PIXY2::pixy2_getFeatures();                                        // On appelle la fonction de traitement.
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getAllFeature (Byte features){
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetLineFeature(1, features);                       // On envoie la trame de demande de suivi de ligne
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            cr = PIXY2::pixy2_getFeatures();                                        // On appelle la fonction de traitement.
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setMode (Byte mode)
{
    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetMode (mode);                                    // On envoie la trame de règlage du mode de fonctionnement du suivi de ligne
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setNextTurn (sWord angle)
{
    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetNextTurn (angle);                               // On envoie la trame de choix de l'angle du prochain virage
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setDefaultTurn (sWord angle)
{
    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetDefaultTurn (angle);                            // On envoie la trame de choix de l'angle par défaut des virages
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_setVector (Byte vectorIndex)
{
    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndSetVector (vectorIndex);                           // On envoie la trame de choix du vecteur à suivre
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}


PIXY2::T_pixy2ErrorCode PIXY2::pixy2_ReverseVector (void)
{
    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;

    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndReverseVector ();                                  // On envoie la trame d'inversion de l'image (haut en bas et bas en haut)
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;

        case dataReceived :                                                         // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if ((msg->pixType == PIXY2_REP_ACK) || (msg->pixType == PIXY2_REP_ERROR)) {
                                                                                    // On vérifie que la trame est du type convenable (ACK ou ERREUR)
                cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];                  // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
            } else cr = PIXY2_TYPE_ERROR;                                           // Si le type ne correspond à rien de normal on signale une erreur de type.
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}

PIXY2::T_pixy2ErrorCode PIXY2::pixy2_getRGB (Word x, Word y, Byte saturate, T_pixy2Pixel **pixel){

    T_pixy2RcvHeader    *msg = (T_pixy2RcvHeader*) &Pixy2_buffer[hPointer];
    T_pixy2ErrorCode    cr = PIXY2_OK;
    
    switch (etat) {
        case idle :                                                                 // Si la caméra est inactive
            wPointer = 0;                                                           // On remonte en haut du buffer
            cr = PIXY2::pixy2_sndGetRGB(x, y, saturate);                            // On envoie la trame de demande de la couleur (RGB) d'un carée de pixel
            if (cr!= PIXY2_OK) return cr;                                           // S'il y a une erreur lors de l'envoi on ejecte !
            etat = messageSent;                                                     // On passe à l'attente du message de réponse
            cr = PIXY2_BUSY;                                                        // On signale à l'utilisateur que la caméra est maintenant occupée
            break;
            
        case dataReceived :                                                      // Quand on a reçu l'intégralité du message
            if (frameContainChecksum) {                                             // Si la trame contient un checksum
                if (pixy2_validateChecksum (&Pixy2_buffer[hPointer]) != 0) {        // On lance la validation du checksum
                    return PIXY2_BAD_CHECKSUM;                                      // Si le checksum est faux on retourne une erreur
                }
            }
            if (msg->pixType == PIXY2_REP_ACK) {                                    // On vérifie que la trame est du type convenable (REPONSE ACK)
                *pixel = (T_pixy2Pixel*) &Pixy2_buffer[dPointer];                    // On mappe le pointeur de structure sur le buffer de réception.
            } else {                                                                // Si ce n'est pas le bon type
                if (msg->pixType == PIXY2_REP_ERROR) {                              // Cela pourrait être une trame d'erreur
                    cr = *(T_pixy2ErrorCode*) &Pixy2_buffer[dPointer];              // Si c'est le cas, on copie le code d'erreur reçu dans la variable de retour
                } else cr = PIXY2_TYPE_ERROR;                                       // Si le type ne correspond à rien de normal on signale une erreur de type.
            }
            etat = idle;                                                            // On annoce que la pixy est libre
            break;

        default :                                                                   // Dans tous les autres cas
            cr = PIXY2_BUSY;                                                        // On signale que la caméra est occupée.
            break;
    }
    return cr;
}



PIXY2::T_pixy2ErrorCode PIXY2::pixy2_validateChecksum (Byte* tab){
    Word    i, sum = 0;
    T_Word  *tmp;
    
    for (i=0; i<*(tab+3);i++)   sum = sum + *(tab + PIXY2_CSHEADERSIZE + i);
    tmp = (T_Word*) (tab+4);
    
    if (tmp->mot == sum) return PIXY2_OK;
    else {
        if (_DEBUG_) {
            sommeDeControle = sum;
            sommeRecue = tmp->mot;
        }
        return PIXY2_BAD_CHECKSUM;
    }
} 
