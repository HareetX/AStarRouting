from dataclasses import dataclass, field

from typing import Dict


@dataclass
class NetClass:
    """The ``project`` token defines a KiCad layout according to the project file format used in
        ``.kicad_pro`` files.

        """
    bus_width: str = ""
    """The ``bus_width`` token defines the netClass"""

    clearance: str = ""
    """The ``clearance`` token defines the netClass"""

    diff_pair_gap: str = ""
    """The ``diff_pair_gap`` token defines the netClass"""

    diff_pair_via_gap: str = ""
    """The ``diff_pair_via_gap`` token defines the netClass"""

    diff_pair_width: str = ""
    """The ``diff_pair_width`` token defines the netClass"""

    microvia_diameter: str = ""
    """The ``microvia_diameter`` token defines the netClass"""

    microvia_drill: str = ""
    """The ``microvia_drill`` token defines the netClass"""

    name: str = ""
    """The ``name`` token defines the netClass"""

    pcb_color: str = ""
    """The ``pcb_color`` token defines the netClass"""

    schematic_color: str = ""
    """The ``schematic_color`` token defines the netClass"""

    track_width: str = ""
    """The ``track_width`` token defines the netClass"""

    via_diameter: str = ""
    """The ``via_diameter`` token defines the netClass"""

    via_drill: str = ""
    """The ``via_drill`` token defines the netClass"""

    wire_width: str = ""
    """The ``wire_width`` token defines the netClass"""

    def from_sexpr(self, exp: list):
        """Convert the given S-Expresstion into a Board object

                Args:
                    - exp (list): Part of parsed S-Expression ``(kicad_pcb ...)``

                Raises:
                    - Exception: When given parameter's type is not a list
                    - Exception: When the first item of the list is not kicad_pcb

                Returns:
                    - Board: Object of the class initialized with the given S-Expression
                """
        if not isinstance(exp, list):
            raise Exception("Expression does not have the correct type")

        # if exp[0] != 'kicad_pcb':
        #     raise Exception("Expression does not have the correct type")

        object = self
        for i in range(len(exp)):
            item = exp[i]
            if item == '"bus_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.bus_width = v
            if item == '"clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.clearance = v
            if item == '"diff_pair_gap":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.diff_pair_gap = v
            if item == '"diff_pair_via_gap":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.diff_pair_via_gap = v
            if item == '"diff_pair_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.diff_pair_width = v
            if item == '"line_style":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.line_style = v
            if item == '"microvia_diameter":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.microvia_diameter = v
            if item == '"microvia_drill":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.microvia_drill = v
            if item == '"name":': object.name = exp[i + 1].strip('",')
            if item == '"pcb_color":':
                object.pcb_color = exp[i + 1].lstrip('"') + '(' + exp[i + 2] + ' ' + exp[i + 3] + ' ' \
                                   + exp[i + 4] + ' ' + str(exp[i + 5]) + ')'
            if item == '"schematic_color":':
                object.schematic_color = exp[i + 1].lstrip('"') + '(' + exp[i + 2] + ' ' + exp[i + 3] + ' ' \
                                         + exp[i + 4] + ' ' + str(exp[i + 5]) + ')'
            if item == '"track_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.track_width = v
            if item == '"via_diameter":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.via_diameter = v
            if item == '"via_drill":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.via_drill = v
            if item == '"wire_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.wire_width = v

        return object


@dataclass
class NetSetting:
    """The ``project`` token defines a KiCad layout according to the project file format used in
        ``.kicad_pro`` files.

        """
    classes: Dict[str, NetClass] = field(default_factory=dict)
    """The ``classes`` token defines the netClass"""

    netClassPatterns: Dict[str, str] = field(default_factory=dict)
    """The ``classes`` token defines the netClass"""

    def from_sexpr(self, exp: list):
        """Convert the given S-Expresstion into a Board object

                Args:
                    - exp (list): Part of parsed S-Expression ``(kicad_pcb ...)``

                Raises:
                    - Exception: When given parameter's type is not a list
                    - Exception: When the first item of the list is not kicad_pcb

                Returns:
                    - Board: Object of the class initialized with the given S-Expression
                """
        if not isinstance(exp, list):
            raise Exception("Expression does not have the correct type")

        # if exp[0] != 'kicad_pcb':
        #     raise Exception("Expression does not have the correct type")

        exp_ = []
        flag = False
        items = []
        for item in exp:
            if item == '[':
                flag = True
            elif ']' in item:
                flag = False
                exp_.append(items)
                items = []
            else:
                if item == ',':
                    pass
                elif flag:
                    items.append(item)
                else:
                    exp_.append(item)

        object = self
        for i in range(len(exp_)):
            item = exp_[i]
            if item == '"classes":':
                for net_class in exp_[i + 1]:
                    net_class_ = NetClass().from_sexpr(net_class)
                    object.classes[net_class_.name] = net_class_
            if item == '"netclass_patterns":':
                for net_with_class in exp_[i + 1]:
                    object.netClassPatterns[net_with_class[3]] = net_with_class[1].strip('",')

        return object
