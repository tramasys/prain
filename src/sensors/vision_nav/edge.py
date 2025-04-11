import math

class EdgeCandidate:   
    def __init__(self, line: tuple[int, int, int, int], center: tuple[int, int]):
        x1, y1, x2, y2 = line
        cx, cy = center
        cx = cx // 2
        cy = cy // 2

        # Abstand der Punkte zum Zentrum
        d1 = (x1 - cx) ** 2 + (y1 - cy) ** 2
        d2 = (x2 - cx) ** 2 + (y2 - cy) ** 2

        # Immer: von näherem Punkt zum weiter entfernten Punkt
        if d1 <= d2:
            self.__x1, self.__y1 = x1, y1  # näher
            self.__x2, self.__y2 = x2, y2  # weiter weg
        else:
            self.__x1, self.__y1 = x2, y2
            self.__x2, self.__y2 = x1, y1

        self.__angle = self.get_angle()
        
    def get_angle(self) -> int:
        dx = self.__x2 - self.__x1
        dy = self.__y2 - self.__y1

        angle_rad = math.atan2(-dy, dx)  # flip Y axis to match unit circle
        angle_deg = (360 - ((math.degrees(angle_rad) - 90 + 360) % 360)) % 360
        # angle_deg = (math.degrees(angle_rad) - 90 + 360) % 360


        return round(angle_deg)
    
    def get_line(self):
        return self.__x1, self.__y1, self.__x2,  self.__y2
    
    def __str__(self):
        return str(self.__x1, self.__y1, self.__x2, self.__y2, self.__angle)
    
    import math

    def debug_angle(self, x1, y1, x2, y2, cx, cy):
        # Abstand zum Zentrum berechnen
        d1 = (x1 - cx)**2 + (y1 - cy)**2
        d2 = (x2 - cx)**2 + (y2 - cy)**2

        if d1 <= d2:
            sx, sy = x1, y1
            ex, ey = x2, y2
        else:
            sx, sy = x2, y2
            ex, ey = x1, y1

        dx = ex - sx
        dy = ey - sy

        angle_rad = math.atan2(-dy, dx)
        angle_deg = (math.degrees(angle_rad) + 360) % 360

        # Richtung grob als Emoji
        direction = ""
        if 337 <= angle_deg or angle_deg < 23:
            direction = "⬆️"
        elif 23 <= angle_deg < 68:
            direction = "↗️"
        elif 68 <= angle_deg < 113:
            direction = "➡️"
        elif 113 <= angle_deg < 158:
            direction = "↘️"
        elif 158 <= angle_deg < 203:
            direction = "⬇️"
        elif 203 <= angle_deg < 248:
            direction = "↙️"
        elif 248 <= angle_deg < 293:
            direction = "⬅️"
        elif 293 <= angle_deg < 337:
            direction = "↖️"

        print(f"Line: ({x1},{y1}) → ({x2},{y2}) | From: ({sx},{sy}) → ({ex},{ey}) | dx={dx} dy={dy} | angle={round(angle_deg)}° {direction}")

    
    
class Edge: 
    def __init__(self, angle: int):
        self.__angle = angle
        
    def get_angle(self) -> int:
        return self.__angle


    
