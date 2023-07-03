from dataclasses import dataclass, field

from typing import Dict


@dataclass
class DesignRules:
    """The ``project`` token defines a KiCad layout according to the project file format used in
            ``.kicad_pro`` files.

            """
    max_error:  str = ""

    min_clearance:  str = ""

    min_connection: str = ""

    min_copper_edge_clearance: str = ""

    min_hole_clearance: str = ""

    min_hole_to_hole: str = ""

    min_microvia_diameter: str = ""

    min_microvia_drill: str = ""

    min_resolved_spokes: str = ""

    min_silk_clearance: str = ""

    min_text_height: str = ""

    min_text_thickness: str = ""

    min_through_hole_diameter: str = ""

    min_track_width: str = ""

    min_via_annular_width: str = ""

    min_via_diameter: str = ""

    solder_mask_to_copper_clearance: str = ""

    use_height_for_length_calcs: str = ""

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
            if item == '"max_error":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.max_error = v
            if item == '"min_clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_clearance = v
            if item == '"min_connection":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_connection = v
            if item == '"min_copper_edge_clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_copper_edge_clearance = v
            if item == '"min_hole_clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_hole_clearance = v
            if item == '"min_hole_to_hole":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_hole_to_hole = v
            if item == '"min_microvia_diameter":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_microvia_diameter = v
            if item == '"min_microvia_drill":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_microvia_drill = v
            if item == '"min_resolved_spokes":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_resolved_spokes = v
            if item == '"min_silk_clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_silk_clearance = v
            if item == '"min_text_height":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_text_height = v
            if item == '"min_text_thickness":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_text_thickness = v
            if item == '"min_through_hole_diameter":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_through_hole_diameter = v
            if item == '"min_track_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_track_width = v
            if item == '"min_via_annular_width":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_via_annular_width = v
            if item == '"min_via_diameter":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.min_via_diameter = v
            if item == '"solder_mask_to_copper_clearance":':
                value = exp[i + 1].strip('",')
                v = float(value)
                if v.is_integer(): v = int(v)
                object.solder_mask_to_copper_clearance = v
            if item == '"use_height_for_length_calcs":':
                value = exp[i + 1].strip('",')
                if value == 'true':
                    v = True
                elif value == 'false':
                    v = False
                else:
                    v = value
                object.use_height_for_length_calcs = v

        return object

@dataclass
class DesignSetting:
    """The ``project`` token defines a KiCad layout according to the project file format used in
            ``.kicad_pro`` files.

            """
    rules: str = ""
    """The ``rules`` token defines the design rules"""

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

        # exp_ = []
        # flag = False
        # items = []
        # for item in exp:
        #     if item == '[':
        #         flag = True
        #     elif ']' in item:
        #         flag = False
        #         exp_.append(items)
        #         items = []
        #     else:
        #         if item == ',':
        #             pass
        #         elif flag:
        #             items.append(item)
        #         else:
        #             exp_.append(item)

        object = self
        for i in range(len(exp)):
            item = exp[i]
            if item == '"rules":':
                object.rules = DesignRules().from_sexpr(exp[i+1])

        return object