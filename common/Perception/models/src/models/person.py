class Person:
    """ A person class that saves all the information about a person that the robot meets """

    def __init__(self, name, fav_drink):
        self.name = name
        self.fav_drink = fav_drink
        self.age_range = (0, 0)
        self.gender = None
        self.tshirt_colour = None
        self.hair_colour = None
        self.last_known_location = None
        self.introduced_to = []

    def get_fav_coffee(self):
        return self.fav_coffee

    def __str__(self):
        return (
            f"name : {self.name}\n"
            f"fav drink : {self.fav_drink}\n"
            f"age range : {self.age_range}\n"
            f"gender : {self.gender}\n"
            f"hair colour : {self.hair_colour}\n"
            f"t-shirt colour : {self.tshirt_colour}"
        )