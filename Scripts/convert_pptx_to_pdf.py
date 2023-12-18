import os
import comtypes.client

def PPTtoPDF(inputFileName, outputFileName, formatType = 32):
    powerpoint = comtypes.client.CreateObject("Powerpoint.Application")
    powerpoint.Visible = 1

    if outputFileName[-3:] != 'pdf':
        outputFileName = outputFileName + ".pdf"
    deck = powerpoint.Presentations.Open(os.path.abspath(inputFileName))
    deck.SaveAs(os.path.abspath(outputFileName), formatType) # formatType = 32 for ppt to pdf
    deck.Close()
    powerpoint.Quit()

def convert_pptx_to_pdf(input_folder):
    pptx_files = [os.path.join(root, file) for root, dirs, files in os.walk(input_folder) for file in files if file.endswith('.pptx')]

    for pptx_file in pptx_files:
        pdf_file = os.path.splitext(pptx_file)[0] + '.pdf'

        # Convert PowerPoint to PDF with original theme and fonts
        print("Convert:", pptx_file)
        PPTtoPDF(pptx_file, pdf_file)

if __name__ == "__main__":
    input_folder = "."
    convert_pptx_to_pdf(input_folder)
