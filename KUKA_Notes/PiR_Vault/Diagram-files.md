It seems that Diagram-files are needed for the program to connect according to the RSI_EthernetConfig.rsi file. This hower seemsto be internal code, maybe in the RSI_create() function run in the program on the [[KUKA Robot controller(con)|con]]. It is not stated in any code we have acces to, that the Diagram-files would be needed. It is only stated in the RSI Mnaual.
Therefore this file should be used too. However, how does this type of file work?
The Diagram-files hold the RSI-Objects configuration in the RSI-context and so is simply a lot of object in- and out-puts and the connections or edges between them.

Ths simplest example of the Digram-file, where it is made just for listening on the Con's side, looks like so:
```
```