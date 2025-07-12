class FindObjectInRoom:
    def __init__(self, room_name, object_name):
        self.room_name = room_name
        self.object_name = object_name

    def execute(self):
        # Logic to find the object in the specified room
        print(f"Searching for {self.object_name} in {self.room_name}...")
        # Here you would implement the actual search logic
        found = True  # Simulating that the object was found
        if found:
            print(f"{self.object_name} found in {self.room_name}.")
        else:
            print(f"{self.object_name} not found in {self.room_name}.")
