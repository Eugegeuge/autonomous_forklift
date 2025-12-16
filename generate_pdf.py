from fpdf import FPDF

class PDF(FPDF):
    def header(self):
        self.set_font('Arial', 'B', 15)
        self.cell(0, 10, 'Hoja de Ruta: Carretilla Elevadora Autonoma', 0, 1, 'C')
        self.ln(10)

    def footer(self):
        self.set_y(-15)
        self.set_font('Arial', 'I', 8)
        self.cell(0, 10, 'Pagina ' + str(self.page_no()) + '/{nb}', 0, 0, 'C')

    def chapter_title(self, title):
        self.set_font('Arial', 'B', 12)
        self.set_fill_color(200, 220, 255)
        self.cell(0, 6, title, 0, 1, 'L', 1)
        self.ln(4)

    def chapter_body(self, body):
        self.set_font('Arial', '', 11)
        self.multi_cell(0, 5, body)
        self.ln()

pdf = PDF()
pdf.alias_nb_pages()
pdf.add_page()
pdf.set_font('Arial', '', 12)

# Content
pdf.chapter_title('1. Resumen')
pdf.chapter_body(
    "Este proyecto tiene como objetivo desarrollar un sistema de carretilla elevadora autonoma "
    "capaz de navegar por un almacen, aproximarse a objetivos (usando marcadores ArUco) "
    "y realizar operaciones de carga/descarga.\n\n"
    "El sistema utilizara ROS 2 Jazzy y Nav2 Route Server para una navegacion basada en grafos."
)

pdf.chapter_title('2. Arquitectura')
pdf.chapter_body(
    "1. Entorno de Simulacion:\n"
    "   - Actual: Mvsim con Turtlebot3 (Placeholder) y mapa warehouse.\n"
    "   - Futuro: Gazebo con modelo personalizado para fisicas de picking.\n\n"
    "2. Stack de Navegacion (Nav2):\n"
    "   - Route Server: Usa un Grafo de Rutas para definir caminos validos (pasillos).\n"
    "   - Localizacion: AMCL.\n\n"
    "3. Maquina de Estados:\n"
    "   - Aparcado (Idle)\n"
    "   - Navegacion (Route Server)\n"
    "   - Aproximacion (ArUco)\n"
    "   - Carga/Descarga\n"
    "   - Manual\n"
    "   - E-Stop"
)

pdf.chapter_title('3. Pasos de Implementacion')
pdf.chapter_body(
    "Fase 1: Navegacion Basica (Actual)\n"
    "   - [x] Configurar Simulacion (Mvsim + Turtlebot).\n"
    "   - [ ] Configurar Nav2 Route Server.\n"
    "   - [ ] Probar navegacion.\n\n"
    "Fase 2: Percepcion y Aproximacion\n"
    "   - [ ] Implementar deteccion de ArUco y Servoing Visual.\n\n"
    "Fase 3: Maquina de Estados y Logica\n"
    "   - [ ] Implementar Maquina de Estados.\n\n"
    "Fase 4: Manipulacion\n"
    "   - [ ] Implementar picking.\n\n"
    "Fase 5: Seguridad y Manual\n"
    "   - [ ] Implementar E-Stop y Teleop."
)

pdf.output('project_roadmap.pdf', 'F')
print("PDF generated successfully.")
