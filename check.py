import kiss_icp.datasets

print("--- Dataloader Disponibili (stringhe) ---")
available = kiss_icp.datasets.available_dataloaders()
print(available)

print("\n--- Tipi di Dataloader (stringa -> NomeClasse) ---")
types = kiss_icp.datasets.dataloader_types()
print(types)
