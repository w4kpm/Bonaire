#! /bin/bash
mv Bonaire-B.Cu.gbr Bonaire.GBL
mv Bonaire-F.Cu.gbr Bonaire.GTL
mv Bonaire-F.Mask.gbr Bonaire.GTS
mv Bonaire-B.Mask.gbr Bonaire.GBS
mv Bonaire-F.SilkS.gbr Bonaire.GTO
mv Bonaire-B.SilkS.gbr Bonaire.GBO
mv Bonaire-Edge.Cuts.gbr Bonaire.GML
mv Bonaire.drl Bonaire.TXT
zip BonaireCompiled.zip Bonaire.*
