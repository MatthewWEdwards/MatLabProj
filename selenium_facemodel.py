from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time

# Initialize chrome webdriver
options = webdriver.ChromeOptions() 
prefs = {'download.default_directory' : '/media/removable/UNTITLED/python/ee371r/test_downloads'}
options.add_experimental_option('prefs', prefs)
driver = webdriver.Chrome("/usr/lib/chromium-browser/chromedriver", chrome_options=options)
driver.get("http://cvl-demos.cs.nott.ac.uk/vrn/")

# Upload image
upload = driver.find_element_by_name("fileToUpload")
upload.send_keys("/media/removable/UNTITLED/python/ee371r/test_upload/obama.jpg")
driver.find_element_by_name("submit").click()

# Grab download
wait = WebDriverWait(driver, 30)
wait.until(EC.title_is("3D Reconstruction from a Single Image"))
time.sleep(5) # TODO: Experiment with this wait time. Waiting is definitely necessary.
obj_download = driver.find_element_by_xpath("//*[@id=\"objlink\"]")
download = obj_download.click()



