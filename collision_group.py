
class CollisionGroup:

    relation = list()
    for i in range(15):
        relation.append(list())

    index_dict = {
        "env": 0,
        "player": 1,
        "ball": 2,
        "aim_line": 3,
        "target": 4,
    }

    @classmethod
    def get_index(cls, name):
        return cls.index_dict[name]

    @classmethod
    def get_group(cls, name):
        return 2 ** cls.get_index(name)

    @classmethod
    def get_mask(cls, name):
        return sum([2 ** x for x in cls.relation[cls.get_index(name)]])

    @classmethod
    def set_relation(cls, name_1, name_2):
        index_1 = cls.get_index(name_1)
        index_2 = cls.get_index(name_2)
        cls.relation[index_1].append(index_2)
        cls.relation[index_2].append(index_1)

    @classmethod
    def init(cls):
        cls.set_relation("env", "player")
        cls.set_relation("env", "ball")
        cls.set_relation("env", "aim_line")
        cls.set_relation("ball", "target")
        cls.set_relation("aim_line", "target")
