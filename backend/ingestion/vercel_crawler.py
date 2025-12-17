import requests
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from typing import List
from backend.ingestion.models import InputContent, Chapter, TextSection
import logging

logger = logging.getLogger(__name__)

class VercelCrawler:
    def __init__(self, base_url: str = "https://ai-and-robotics.vercel.app/"):
        self.base_url = base_url
        self.sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"

    def crawl(self) -> InputContent:
        urls = self._fetch_sitemap_urls()
        chapters = []
        
        for url in urls:
            try:
                logger.info(f"Crawling {url}...")
                response = requests.get(url)
                if response.status_code != 200:
                    logger.warning(f"Failed to fetch {url}: {response.status_code}")
                    continue
                
                soup = BeautifulSoup(response.content, 'html.parser')
                
                # Extract Title
                title = soup.title.string if soup.title else url
                
                # Extract Content
                # Docusaurus usually puts content in <main> or <article>
                article = soup.find('article') or soup.find('main')
                if not article:
                    logger.warning(f"No content found for {url}")
                    continue
                
                # Simple extraction: Get all headers and text
                sections = self._parse_sections(article)
                
                if sections:
                    chapters.append(Chapter(
                        chapter_title=title,
                        sections=sections
                    ))
                    
            except Exception as e:
                logger.error(f"Error crawling {url}: {e}")

        return InputContent(
            document_id="vercel-docs-crawl",
            chapters=chapters
        )

    def _fetch_sitemap_urls(self) -> List[str]:
        try:
            response = requests.get(self.sitemap_url)
            if response.status_code != 200:
                logger.error(f"Failed to fetch sitemap: {response.status_code}")
                return []
            
            root = ET.fromstring(response.content)
            # Sitemap namespace usually http://www.sitemaps.org/schemas/sitemap/0.9
            namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
            urls = []
            for url in root.findall('ns:url', namespace):
                loc = url.find('ns:loc', namespace).text
                # Fix domain mismatch
                if "ai-and-humanoid-robotics.vercel.app" in loc:
                    loc = loc.replace("ai-and-humanoid-robotics.vercel.app", "ai-and-robotics.vercel.app")
                urls.append(loc)
            return urls
        except Exception as e:
            logger.error(f"Error parsing sitemap: {e}")
            return []

    def _parse_sections(self, soup_element) -> List[TextSection]:
        sections = []
        current_title = "Introduction"
        current_text = []
        
        for element in soup_element.descendants:
            if element.name in ['h1', 'h2', 'h3']:
                if current_text:
                    sections.append(TextSection(
                        section_title=current_title,
                        text="\n".join(current_text).strip()
                    ))
                current_title = element.get_text().strip()
                current_text = []
            elif element.name in ['p', 'li', 'code', 'pre']:
                text = element.get_text().strip()
                if text:
                    current_text.append(text)
        
        if current_text:
            sections.append(TextSection(
                section_title=current_title,
                text="\n".join(current_text).strip()
            ))
            
        return sections
